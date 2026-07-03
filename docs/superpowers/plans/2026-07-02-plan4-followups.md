# Plan 4 终审跟进事项(Plan 5~6 输入)

来源:feature/eki-sri-drivers 整分支终审(2026-07-03,结论 APPROVE-FOR-MERGE,干净重建 0 警告、252/252 测试 0 失败、两包 10 二进制 3 连跑无 flaky、零残留)。

计划遗留风险 1~9(`2026-07-02-plan4-eki-sri-drivers.md` 遗留风险节)仍为权威清单,本文不重复其原文;状态对照见 `.superpowers/sdd/plan4-final-review.md` §5。以下为终审新汇总条目。

## 已裁决计划勘误(引用计划原文者注意,勿以原文为准)

1. **Task 8 gtest 目标名全局冲突,已改名 `test_eki_tcp_client_transport`**(T8 评审 D1)
   - 背景:计划原文的 `catkin_add_gtest(test_tcp_client_transport ...)` 与 sri 包同名目标在 CMake 全局命名空间冲突(CMP0002/0079,评审独立复现);实际交付目标/二进制名为 `test_eki_tcp_client_transport`(源文件名不变)。
   - 影响:凡引用计划 Task 8 Step 4/5 原文命令跑该二进制者会找不到目标;计划中"gtest 目标按包隔离"的断言为误。
   - 建议动作:计划文档回写改名与断言更正;Plan 5 若新增全局唯一性存疑的测试目标,一律带包前缀。
   - 归属:计划维护。

2. **`sri_mock_server_main.cpp` 顶部注释与实际行为矛盾**(T5 评审 D2)
   - 背景:注释称 "This binary needs a fixed port",实际监听内核自选端口并打印;计划原文如此,交付逐字保留。
   - 建议动作:Plan 5 为集成/冒烟落地固定端口方案(`--port` 参数)时一并勘误该注释。
   - 归属:Plan 5。

3. **T8 实施报告 2 处笔误级瑕疵**(T8 评审):diff 行号 L112→L111、D1 冲突方向叙述颠倒。随计划勘误一并更正,防后续引用错误叙述。
   - 归属:计划维护(低优先)。

## 终审新发现(本轮独立读代码)

4. **tare 捕获被断线重连打断时 `requestZero` 可能误报成功**(N1)
   - 背景:`SriStreamSession::reset()`(重连钩子)清 `zero_remaining_` 但不清 `last_zero_ok_`;若此前有一次成功 tare,新 tare 捕获期间(默认 100 样本 ≈ 0.4 s)恰好断线并重连,等待中的 `requestZero` 会读到 `zeroActive()==false` + 上次的 `lastZeroAccepted()==true`,以 success 返回——但偏置实未更新。
   - 影响:窄窗口竞态;误报后偏置仍是上次成功值,数据本身一致,但操作者会误以为已按当前工况重新校零。
   - 建议动作:`reset()` 内加 `last_zero_ok_ = false;`(一行),或为"本次捕获"引入代次计数;补一条"捕获中断线→requestZero 返回 false"的用例。
   - 归属:Plan 5(合并本分支后小修,勿在 merge 前改动)。

5. **`connectNow` 失败后 `connect_requested_` 不清除**(N2)
   - 背景:`EkiBridgeRuntime::connectNow` 置位 `connect_requested_`,仅在连接成功时清除;`auto_reconnect=false` 时一次失败的 `/kuka/eki/connect` 调用会让 io 线程此后无限期按 backoff 重试(手动触发语义蜕变为持续自动重连)。
   - 影响:默认配置 `auto_reconnect=true` 下无行为差异;仅显式关闭自动重连的部署受影响。
   - 建议动作:Plan 5 裁决期望语义(单次尝试 vs 触发后持续)并按裁决收敛实现/文档;manager 编排若依赖"手动 connect 一次性失败即止"须先修此处。
   - 归属:Plan 5。

6. **单线程 `ros::spin` 下阻塞服务令话题发布停顿**(N3)
   - 背景:两节点均单线程 spin;`/sri_ft/zero`(默认最长 3 s)与全部 `/kuka/eki/*` 命令服务(最长 response_timeout+1=3 s)在回调内阻塞,期间同节点的 status/state/diagnostics 定时器与其他服务排队。wrench 发布不受影响(rx 线程直发)。
   - 影响:管理通道可接受;但 Plan 5 manager 若以 `/kuka/eki/state` 新鲜度做健康判定,须容忍命令期间最长 ~3 s 的发布间隙,或给节点换 `AsyncSpinner(2)`。
   - 建议动作:Plan 5 bringup 裁决(容忍或多线程 spinner);manager 的新鲜度阈值设计时计入该间隙。
   - 归属:Plan 5。

7. **SriMockServer 把任何含 "AT+GSD" 的输入都当启动命令**(低优先)
   - 背景:mock 的命令匹配是 `search(buf, "AT+GSD")`,`AT+GSD=STOP\r\n` 也会命中并置 streaming=true(驱动 runtime 从不发 STOP,现有测试不受影响)。
   - 建议动作:Plan 5 若让集成测试走 stop 命令路径,先给 mock 补 STOP 分支。
   - 归属:Plan 5(条件触发)。

## 留待人工/联调的冒烟

8. **Task 6 Step 7 SRI roscore 冒烟未执行(计划明示非门槛)**
   - 内容:`rostopic hz /sri_ft/wrench_raw` ≈ 250、`rostopic echo --offset` 毫秒级、`/sri_ft/zero` 后 fz 归零、status `streaming: True`(对 `sri_mock_server`)。
   - 建议动作:并入 Plan 5 bringup 冒烟清单一次性执行(Task 10 的 EKI 冒烟评审已独立复现,无欠账)。
   - 归属:人工(随 Plan 5)。

9. **真机联调核对项:`rostopic echo --offset /sri_ft/wrench_raw` 抽查 stamp 与到达时刻差**
   - 背景:计划验收清单明文"记入 Plan 5 联调核对单"(Plan 3 跟进 3 的收尾);stamp 为接收时刻而非采样时刻(计划遗留风险 4),不要基于 stamp 做样本间隔统计(用 `package_gaps`)。
   - 归属:人工(Plan 5 联调核对单)。

## 承计划遗留风险中需 Plan 5 主动闭环的三条(点名,防漏)

10. **RSI 侧 latched fault 复位路径尚不存在**(计划遗留风险 3 / 待确认 10):Plan 5 manager 恢复流程编排前必须在 `kuka_rsi_hw_interface` 补 `clearFault()`(保留累计计数,勿用 `RsiSessionMonitor::reset()`)。
    - 归属:Plan 5。

11. **KRL 模板/EkiConfig.xml 逐字段对齐 + 周期心跳义务**(计划遗留风险 2/8):`eki_frame.h` 的 schema 注释是唯一权威;KRL 模板必须实现周期 `EKI_Send`(建议 100 ms),否则 `state_fresh` 空闲期恒 false(诊断恒 WARN)。`/sri_ft/zero` 仅限非 SERVOING 状态调用的约束(遗留风险 5)也在 manager 状态机落实。
    - 归属:Plan 5。
