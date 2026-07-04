# 工具坐标系顺应变换 — 跟进事项(整分支终审,真机部署前必读)

日期:2026-07-04
来源:feature/tool-frame-compliance 整分支终审(APPROVED 前置修复后)
关联:docs/superpowers/specs/2026-07-04-tool-frame-compliance-design.md

## 部署检查单(非阻塞,真机操作项)

1. **过期工具角风险**:`ToolSample.valid` 永不过期,且 `ekiStateCb` 丢弃
   disconnected 帧——若 EKI 断开期间在示教器上改了 `$TOOL`,下次激活会静默
   锁存旧值。**每次伺服启动前:确认 EKI 已连接,且 rosout 中
   "FORCE_COMPLIANCE frame locked" 回显的 tool A/B/C 与示教器一致。**

2. **旧版本行为差异(即使不配新参数)**:非单位姿态下 RKorr 方向与旧构建不同
   (这是本次修复的缺陷 2:修正量此前误按传感器系直接下发 BASE)。首次拖动
   试验前向操作者说明:同样的推力,机器人运动方向可能与旧版本不一致——新行为
   才是正确的。

3. **mount 参数审计**:`sensor_to_flange_abc` 长度错误仅在 init 时 `ROS_WARN`
   一次并回退单位阵;每次激活的 `ROS_INFO` 回显(见勘误)是运行期唯一审计点,
   部署时留意日志。

4. **发布的 wrench 坐标系**(Task 5 审查 nit):`publishState` 发布的
   compensated wrench 保持传感器系,README 未显式声明——若 web 端要画力曲线,
   注意它不是工具系/BASE 系。

## 诊断标记勘误(已裁决,见 spec 顶部)

`ModeState` 不加字段;降级(无 EKI 工具数据 / gravity_n==0)通过每次激活的
`ROS_WARN` + 锁存值 `ROS_INFO` 呈现。UI 若需显示降级状态,后续单独立项
(需同步 29 个 web 测试)。
