# MuJoCo 验证环境 — 部署期 followups

新包 `kuka_mujoco_sim` 是回环仿真验证环境,与真机之间的已知差距。真机/长时运行前复核。
仿真通过 ≠ 真机通过 —— 另见 `2026-07-04-tool-frame-compliance-followups.md`。

## 运行时 / 部署

1. **SRI `sendall` 在锁内对阻塞 socket 调用**
   `sri_endpoint.py::send_frame_if_streaming` 与 ACK 路径在持有 `self._lock` 时对 client fd 调 `sendall`。
   回环 + 31 字节帧下已验证不阻塞(14/14 无 flaky)。但真机 `sri_ft_driver` 若慢/卡使其 TCP 发送缓冲填满,`sendall` 会持锁阻塞 sim 循环,并使 `stop()` 的 `join` 超时后卡在 `_lock`。
   **修**:给 client fd 设发送超时,或 snapshot `conn` 后在锁外 `sendall`(`recv` 已在锁外)。

2. **sim_node RSI recv 超时 = 50ms(计划伪码写 8ms)**
   `sim_node.py` 用 50ms `recvfrom` 超时。正常握手后每周期即时返回;50ms 仅在启动握手与真实丢包(故障路径)时生效,超时则 `pose_target6` 保持不跳变。可接受;真机若要严格 4ms 节律可收紧。

3. **RSI 回复单周期严格匹配**
   `rsi_endpoint.apply_reply` 在 `ipoc != awaited_ipoc` 时丢弃该 RKorr 并计 `ipoc_error`。250Hz 下迟到一周期的 `<Sen>` 被丢弃而非补匹配。冒烟回环无碍;真机高负载若偶发迟到需评估。

4. **`_default_model` 依赖 devel-space `__file__` 解析**
   `sim_node.py::_default_model` 由 `dirname(__file__)/../../models/...` 定位 MJCF;`models/` 未被 `CMakeLists.txt` install。纯 install-space 部署会解析失败 —— 显式传 `~model_path`,或在 CMake 加 `install(DIRECTORY models/ ...)`。

## 物理 / 建模

5. **墙接触量级偏大(≈458N @ 10mm 压深)**
   默认墙 `solref/solimp` 刚;当前所有断言只需"接触出现"(>2N)故不受影响。若要更物理的接触量级,调 `models/kuka_tcp_scene.xml` 墙的 `solref/solimp` 或压深(勿改断言语义)。

6. **post-compile override 的 sameframe 陷阱(已修两处,举一反三)**
   MuJoCo 编译器对 identity 位姿的 body/site/geom 会置 `*_sameframe=1`,使 load 后对 `body_ipos`/`site_quat` 等的编辑被静默忽略。已修 `site_sameframe`(mount_abc_deg)与 `body_sameframe`(payload_com_m),均有回归守卫(`test_mount_override.py`,回滚即失败验证过)。**今后新增任何 post-compile 模型字段编辑,务必检查对应 `*_sameframe` 并加物理效应回归,勿写同义反复断言。**

## 标定关联

7. **site_sameframe 潜伏 bug 的历史含义**
   在 T8 修复前,`mount_abc_deg` 对所有角度静默 no-op。此前任何"假设 FT 传感器 mount 旋转已建模"的推断实为无效 —— 与 mount 标定 / tool-frame followups 关联,真机标定前复核。
