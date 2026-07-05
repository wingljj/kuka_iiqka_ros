# MuJoCo 关栈泄漏 + 独立 viewer 修复 — followups (2026-07-05)

承接 `2026-07-05-mujoco-force-loop-fix-followups.md`。这一轮的起因是用户报告"参数直通修复后，GUI 里拖末端仍不动、e2e 有时 PASS 有时 NOT-RUN"。系统化调试后定位到**两层根因**，与力顺应算法本身无关。

## 根因一：残留僵尸进程霸占单客户端 EKI mock（"反复不动"主因）

- `pkill -f "devel/lib/"` 在本机**杀不净**编译的 C++ 节点（`eki_bridge_node` / `soft_robot_manager` 等）。一个存活 34 分钟的僵尸 `eki_bridge_node`（上一次未干净关闭的 roslaunch 残留）一直连着 EKI mock。
- EKI mock 是**单客户端**服务器（`eki_mock_server.cpp`: `listen(fd, 1)`，单个 `client_fd_`）。僵尸 bridge 占住那唯一槽位 → 新栈的 bridge 完成 TCP 连接但**永不被 mock 服务**（mock Recv-Q 堆积、新连接半开）→ `eki_connected` 恒 false → 系统永远到不了 READY → start_servo 被拒 → 不动。
- 已证实：杀掉僵尸后立即恢复，headless e2e 稳定 PASS +69.9mm（背靠背两次）。
- **修复**：`e2e_compliance_smoke.py` 用 `start_new_session=True` 起栈、关栈时对**整个进程组** SIGINT→(超时)SIGKILL，不再遗留孤儿。背靠背 e2e 已验证无泄漏。
- **手动 roslaunch 仍需注意**：Ctrl-C 未必收干净所有 C++ 子节点。可靠清理见 README "Still not moving? checklist" 第 1 步（按可执行名 `ps|grep|kill -9`，勿依赖 `pkill -f`）。
- **待办**：可在 `mujoco_sim.launch` 或一个 wrapper 里加"启动前端口/残留自检"，或给 EKI mock 支持多客户端 / 启动时踢掉旧连接（**要改现有包**，本次零改范围外）。

## 根因二：GUI 下 viewer 抢 CPU/GPU 触发控制器硬超时（`gui:=true` 确定性不动）

- 单变量对照（同一干净起点，只切 `gui`）：headless → 法兰 **+69.9mm**（PASS）；`gui:=true` → **+0.00mm**（FAIL），本机稳定复现。
- 第一版把 viewer 内嵌在 sim_node 的 250Hz 循环里，`viewer.sync()` 渲染停顿直接饿死 RSI UDP → hw_interface 连续 5 次 8ms 读超时闩锁 fault → DEGRADED。**已改为独立 `viewer_node` 进程**（订阅 `flange_pose`，`mj_forward` 纯运动学镜像，不碰 RSI/SRI socket）——消除了"闩锁 fault"，READY 能稳定达成。
- **但没根治**：独立进程仍与 sim_node 抢同机 CPU/GPU。实测 gui 下 SRI `wrench_raw` 最大帧间隔 **~15.4ms > 控制器 `wrench_timeout_s=12ms`**，触发 `force_compliance_core.cpp:48` 的陈旧-wrench 零输出路径（`law_.reset()`），压制顺应运动。RSI 也偶发超时。
- 这是**软实时仿真主机 vs 硬实时控制器**的固有矛盾，非算法缺陷。**权威验证是 headless e2e**（稳定）。
- **待办（若需可靠 GUI）**，按代价排序：
  1. viewer 进程 `nice`/`SCHED_IDLE` 降优先级 + `taskset` 绑到独立核，减少对 sim_node 的抖动（新包内可做，最小侵入）。
  2. 用 rosbag 录 `flange_position`/`flange_pose` 后离线回放渲染（完全解耦，推荐给"只想看运动"的场景）。
  3. 放宽控制器 `wrench_timeout_s`（**改现有包**，且会削弱真机安全语义，不建议为仿真观看而改）。

## 本轮改了哪些文件（均在解耦新包 + docs，零改现有控制器/驱动包）

- `launch/mujoco_sim.launch`：`gui:=true` 由"内嵌 viewer 参数"改为**条件起独立 `viewer_node`**（`if="$(arg gui)"`），remap `flange_pose`。
- `src/kuka_mujoco_sim/sim_node.py`：移除内嵌 viewer；新增 `~flange_pose`（PoseStamped, SI）发布供 viewer 用；spin 循环不再有任何渲染调用。
- `src/kuka_mujoco_sim/mujoco_world.py`：新增 `get_flange_pose_m()`（读位姿）+ `set_flange_pose_m()`（viewer 侧置位姿后 `mj_forward` 纯运动学，无 dynamics 故不与主 sim 分岔）。
- `src/kuka_mujoco_sim/viewer_node.py`（新）+ `scripts/viewer_node`（新，rosrun 入口）：独立渲染节点，顶部 docstring 标注 GUI 局限。
- `CMakeLists.txt`：install `scripts/viewer_node`。
- `test/e2e_compliance_smoke.py`：进程组起停（`start_new_session` + `killpg` SIGINT→SIGKILL）根治关栈泄漏；`--gui` 开关透传。
- `README.md`：层 3 改为"headless 为权威验证"；新增"Still not moving? checklist"（僵尸清理为第 1 步）+ "GUI 局限"小节。
