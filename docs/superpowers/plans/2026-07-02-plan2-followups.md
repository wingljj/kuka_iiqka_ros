# Plan 2 终审跟进事项(Plan 3~5 输入)

来源:feature/rsi-hw-interface-msgs 整分支终审(2026-07-02,结论 APPROVE,110/110 测试、零警告、干净重建复验)。

## Plan 3(soft_robot_controllers)

1. **控制器 dt 用实测 `period`**,不要写死 0.004——RSI 节拍由 KRC 主导。
2. JointState 相关测试需给 `RsiMockCore` 补 `setJointAngles`(现 AIPos 恒零,setter 需仿 setPose 自行添加)。
3. **故障感知方式待定**:控制器无法经硬件接口感知 fault/connected(未实现可选的 RobotModeStateInterface);选项 = 订阅 `/kuka/rsi/state`(非 RT)或补接口。
4. (承 Plan 1)AutoReTare 边沿触发、两路 correction 先求和再过 SafetyLimiter、编译期常量对齐断言(sfc 枚举 × ModeCommand)在控制器包落地。

## Plan 4/5(EKI/manager/bringup)

5. `resetFault()` 接线(EKI/manager);注意 `RsiSessionMonitor::reset()` 会清空累计计数器,与 RsiState.msg "cumulative since node start" 注释矛盾——接线时二选一。
6. KRC 侧 RSI 配置须匹配 `Sen Type="ROS"` + RKorr/Stop/Watchdog/IPOC 字段集。
7. bringup 固定 `listen_ip` 到与 KRC 同网段网卡;可顺手:socket 发送设 MSG_DONTWAIT、修 sim server `--help` 退出码(现为 1)。

## 真机联调核对单

8. 人工核对 IPOC 增速(驱动不校验步长)。
9. 确认 KRC 对"超时周期重发的陈旧 IPOC 回显帧"的丢弃行为(mock 侧计为 echo_err,属诊断噪声)。
10. tinyxml2 read() 内部分配为已声明的唯一实时性偏差;真机 4ms 周期实测超限再换手写解析器。

## 已裁决计划勘误(Plan 3 编写者注意,勿以计划原文为准)

- 计划 L1861-1863:CartesianStateInterface 空类体与自带测试矛盾 → 实现加 claim() no-op 重写(Noetic 基类 claim() 无条件记录)。
- 计划 Task 8 测试 `*j.getPosition()` → `j.getPosition()`(Noetic 按值返回 double)。
