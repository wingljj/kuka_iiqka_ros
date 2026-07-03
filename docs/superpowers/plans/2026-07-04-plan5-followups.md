# Plan 5 整分支终审跟进项(2026-07-04)

终审结论:APPROVE-FOR-MERGE(`.superpowers/sdd/plan5-final-review.md`,分支 feature/manager-calibration-bringup,f79f387..845b5db,13 commits,整仓 296/0/0)。以下为移交 Plan 6 / 装机阶段的跟进清单;标注【必闭环】者在 Plan 6 计划编写时必须逐项吸收或明确排除。

## Important(2)

1. 【必闭环】**I-1 真机 RSI 活动期/停止链路跨层语义缺口**:KRL 模板 MOVECORR 阻塞期心跳停发 → `state_fresh=false` → 真机 SERVOING 确定性落 DEGRADED(闲态 RSI 期落 OFFLINE);且 STOP_RSI 在阻塞期不可达、PC 侧无干净停止触发器(Stop S 仅置 fault、stopServo 忽略 ekiStopRsi 失败)、RSI 流终止后 hw 5 拍闩锁 fault——干净停止真机的结局是 OFFLINE 或 FAULT,与 commissioning_checklist Stage 2/3 "clean READY" 书面预期矛盾。失效方向全部安全(虚报、无非指令运动),仅真机可触发。方案方向(KRL 心跳移入 RSI 活动期旁路 / EKI 状态语义分层 / 核对单预期改写)留 Plan 6 与装机实测裁决;装机前至少打核对单文字补丁对齐预期。关联:Task 9 评审 MOVECORR 装机观察项。
2. 【必闭环】**I-2 startServo 终裁无回滚**:`ManagerRuntime::startServo` 的 `requestStart` 终裁位于 START_RSI(最长 5s 等待)+ 控制器 switch + mode 发布之后,拒绝时无回滚——窗口内 READY 丢失则控制器带激活 mode 运行、RSI 已启而状态回 READY。小改方向:switch 前预裁,或拒绝分支执行 stopServo 等价回滚。

## Minor(6,终审裁决无一须合前修)

3. Task 1 微瑕:`countMiss` 旧注释未同步(kuka_rsi_hw_interface)。
4. 标定 action feedback 相位注释与实际 SOLVING/DONE 发布时机不符(注释级)。
5. 标定 preempt 分支用 `setAborted` 而非 `setPreempted`(语义微偏,客户端可见状态码)。
6. TcpClientTransport 副本"三处差异"口径:实物尚有注释行差异,归一化 diff 口径应记为"guard/include/namespace + 注释",或抹平注释。
7. manager 单独重启后 `controllers_loaded` 永假(spawner 已退出,list_controllers 满足但标志不刷新)——计划遗留风险 2 的实证重申,Plan 6 web 界面若展示该位需注意。
8. bringup README 历史段仍述及已删除的 switch_controller_filter(仅历史记录性文字,无引用)。

## 计划文本勘误(维护性,改 plan 文档时顺手)

9. Plan5 Task 5 N2:"最后样本 bug ⇒ G 偏离/r2<1"机理描述不准(均匀偏移被 bias 吸收;实际由 bias_fx 与 samples_collected 断言钉死)。
10. Plan5 Task 6 勘误 A/B/C:Step 2 RED 预期文字;计划头图 persistPayload/publishManagerState 陈旧命名;头图 start_servo 顺序与决策 4 矛盾。
11. Plan5 Task 10 ④:计划 grep 统计命令元素级双计(实测恰 2 倍),正确做法锚定 `<testsuites>` 根元素。
12. Task 9 备案:KRL 心跳实测 100~110ms(桥侧 1s 阈值 ~9 倍裕量,模板逐字不改)。

## 执行沉淀(约定,已在台账)

- 整仓用例基线更新:296(52+59+8+61+44+39+33);8b/8c 为评审裁决的计划外债务回收任务先例(缺陷经源码+实测双确认后,任务书即契约、TDD 钉死、独立评审同规格)。
