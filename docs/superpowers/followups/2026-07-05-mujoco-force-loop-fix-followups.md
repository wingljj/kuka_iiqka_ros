# MuJoCo 力顺应参数直通修复 — followups (2026-07-05)

在解耦新包 `kuka_mujoco_sim` 内修复"读到力但不动"(`feature/mujoco-force-loop-fix`,零改现有控制器)后的已知边界。
承接 `2026-07-04-mujoco-validation-followups.md`;仿真通过 ≠ 真机通过。

## 修了什么

`mujoco_sim.launch` 原先只把 `payload_mass`/`mount_abc` 喂给 MuJoCo 节点,没喂控制器。
控制器 `gravity_n=0`(未标定)→ 不扣负载自重 → 常驻净力(2kg@`[0,90,0]` 甩到 +X 的 19.62N)
被自适应死区学到 ~24.6N → 手推过不去 → `RKorr=0` → 不动。
修复:launch 在 load `soft_robot_controllers.yaml` **之后**注入
`/force_compliance_controller/payload/gravity_n = payload_mass*9.81` 与
`sensor_to_flange_abc = mount_abc`(逐字直通,已证等价)。
e2e 冒烟实测:40N 拖拽 5s,法兰 +X 位移 ≈ 70mm(PASS)。

## 仍未在环验证 / 边界

1. **COM 偏置未在闭环验证**
   本补丁只直通 `gravity_n` + `sensor_to_flange_abc`,**未**注入 `payload/com_x/y/z`。
   MuJoCo 里 FT sensor site 在 payload 原点,COM 偏置≈0,当前重力力矩项可忽略,故主链闭合不受影响。
   但"COM 偏置产生的重力力矩被正确扣除"这条**未在环验证过**(离线 `test_mount_override.py` 只验了 MuJoCo 侧
   `body_ipos` 确实移动 COM,没接控制器扣力矩)。真机标定会同时定 com+bias;若要在环验证,需给 launch 再直通
   `payload_com_m`(MuJoCo)+ `payload/com_*`(控制器)并断言净力矩≈0。

2. **`bias_*` 传感器零偏未建模**
   直通等价于"理想标定的一个子集":`gravity_n` + mount 已定,但 `bias`(FT 零偏)在 MuJoCo 里恒为 0,
   真机非 0。8 位姿标定流程(`samples_per_pose` 等)本身**未在环跑过** —— 冒烟是手动注入已知 `gravity_n`,
   不是让标定服务算出来。真机前须单独验证标定链。

3. **速度型顺应,非 1:1**
   `gain_translation=0.4 (mm/s)/N`、`max_speed=30mm/s`。~35N 净力仅 ~14mm/s。
   盯**绿色 mocap 方块**看缓慢蠕动,别期待抓取式跟手。已在 README「让它真正动起来」写明。

4. **整数列表参数进 `vector<double>`**
   `mount_abc:="[0, 90, 0]"`(整数)经 roslaunch → 参数服务器 → roscpp `param(vector<double>)` 解析正常
   (已由 `frame locked ... mount A/B/C=[0.000 90.000 0.000]` 回显证实),故 launch 未做 float 强制。
   若将来换 roscpp 版本或参数路径导致回退 identity + `sensor_to_flange_abc must have 3 elements` 警告,
   在 launch 用 `$(eval [float(x) for x in arg('mount_abc').strip('[] ').split(',')])` 强制即可(已验证可行)。

5. **e2e 冒烟用 `mount=[0,0,0]` 打主链**
   `e2e_compliance_smoke.py` 固定 `mount=[0,0,0]`+`payload=2.0` 验主链闭合;mount≠0 的方向对齐由离线
   `test/test_gravity_equivalence.py` 覆盖(净力≈0 对 5 个角度,含 `[0,90,0]`/`[0,45,30]`)。
   若要 mount≠0 也跑在环冒烟,把脚本的 mount/drag 轴参数化即可(拖拽方向须随 mount 改)。

6. **参数在控制器 INIT 读取,`rosparam set` 后不重读**
   `loadPayload` 在控制器 init(force_compliance_controller.cpp:81)读 `gravity_n`,
   `sensor_to_flange_abc` 在 :85 读。launch 注入在控制器 spawn 前落到参数服务器故生效;但运行中
   `rosparam set` 再 stop/start 伺服**不会重读** —— 改参数须重启控制器/整个 launch。
