# Plan 3 终审跟进事项(Plan 4~5 输入)

来源:feature/soft-robot-controllers 整分支终审(2026-07-02,结论 APPROVE-FOR-MERGE,干净重建 0 警告、171/171 测试 0 失败、chain 测试 3 连跑无 flaky)。

计划遗留风险 1~9(`2026-07-02-plan3-soft-robot-controllers.md` 遗留风险节)仍为权威清单,本文不重复;以下为终审新汇总条目。

## 已裁决计划勘误(引用计划原文者注意,勿以原文为准)

1. **Task 4 期望值漏算速度夹持**
   - 背景:计划 Task 4 测试期望值推导时未计入 `ComplianceParams.max_speed` 夹持,按原文数值断言必失败。
   - 影响:仅测试期望值;实现(T3 交付)正确。已裁决最小修复:测试参数加 `max_speed=500` 使夹持不生效。
   - 建议动作:计划文档回写勘误注记;后续计划引用该数值推导时须连带速度夹持。
   - 归属:计划维护(随 Plan 4 编写一并回写)。

2. **Task 10 冒烟命令缺 `rostopic pub -s`**
   - 背景:`-r` 模式下不带 `-s` 时 `now` 仅首条求值,后续 wrench stamp 冻结,~12 ms 后恒触发 `wrench_timeout` → 恒 DEGRADED、修正恒零。
   - 影响:仅计划 Step 5 冒烟文档;交付文件逐字节一致。独立冒烟加 `-s` 后已通过。
   - 建议动作:计划文档回写;Plan 5 bringup 文档中的所有 `rostopic pub -r` 含 stamp 的示例一律带 `-s`。
   - 归属:计划维护 + Plan 5 文档。

## Plan 4(SRI 驱动)

3. **wrench `header.stamp` 必须为采样时刻,且勿发零 stamp**
   - 背景:控制器对零 stamp 回退用接收时刻(`force_compliance_controller.cpp:158`);驱动不打戳时超时检测对传输延迟不敏感(承计划遗留风险 6)。
   - 影响:传输阻塞时安全超时可能失效(退化为"回调是否还在跑")。
   - 建议动作:Plan 4 驱动以采样时刻填 stamp 并写进验收;联调时用 `rostopic echo --offset` 抽查 stamp 与到达时刻差。
   - 归属:Plan 4。

## Plan 5(manager / bringup)

4. **RsiState fault 为最新值语义,无自身新鲜度超时**(T7 评审观察 1)
   - 背景:`/kuka/rsi/state` 停发时控制器沿用最后一个 fault 标志;RSI 断链的安全兜底在 Plan 2 HW 层(write 停发修正),控制器层不重复。
   - 影响:manager 若以控制器视角 ModeState 判健康,须知它感知不到"RsiState 话题本身死掉"。
   - 建议动作:Plan 5 manager 对 `/kuka/rsi/state` 自做新鲜度监测(非控制器职责)。
   - 归属:Plan 5。

5. **`mode_seq_` / `goal_seq_` 为非原子生产侧计数器**(T7 观察 2 / T8 观察 2)
   - 背景:依赖"单一订阅回调线程"前提(头文件已注明);多线程 spinner 下两个回调并发会竞态。
   - 影响:controller_manager 节点若改用 AsyncSpinner 多线程,序号可能重复/跳变导致丢请求。
   - 建议动作:Plan 5 bringup 固定 controller_manager 节点为单线程回调队列,并在 launch 注释中写明约束。
   - 归属:Plan 5。

6. **`stopping()` 内 `requestCancel()` 经 `writeFromNonRT` 阻塞锁**(T8 观察 1)
   - 背景:ros_control 的 stopping 可能运行于 CM 实时线程;`writeFromNonRT` 内部持锁。
   - 影响:stopping 非周期路径,当前风险可接受;若 Plan 5 引入 RT 内核/收紧抖动预算则违反约束。
   - 建议动作:届时改原子 cancel 标志(RT 侧读、update 内应用)。
   - 归属:Plan 5(条件触发)。

7. **preempt 与收敛竞态可回报 SUCCEEDED**(T8 观察 3)
   - 背景:executeCb 发 cancel 后、RT 侧应用前恰好收敛,则回报 SUCCEEDED 而非 PREEMPTED;目标确已达成,语义无害。
   - 影响:上层若假设"preempt 必得 PREEMPTED"会误判。
   - 建议动作:Plan 5 标定流程 / Plan 6 UI 把 SUCCEEDED-after-preempt 当正常终态处理。
   - 归属:Plan 5/6。

## 低优先

8. **`publishState` 节流依赖时钟单调**:`time.toSec() - last_pub_s_ < 0.02` 在 sim time 回跳(bag 循环播放)时会停发 ModeState 直至时间追上;真机 wall clock 不受影响。若 Plan 5 离线回放调试需要,加回跳复位。
9. **goal 默认参数为保守投运值**:yaml `goal: p_gain 1.0 / max_speed_dps 5.0 / tol_deg 0.1` 与测试/计划数值自查用的 `p_gain 20 / 7.5 dps` 不同(有意保守)。Plan 5 标定流程投运时按规格 §14 与现场表现调参,勿以测试数值为投运预期。
