# Plan 6 整分支终审跟进项(2026-07-03)

终审结论:APPROVE-FOR-MERGE(`.superpowers/sdd/plan6-final-review.md`,分支 feature/web-interface,601536f..29802da,10 commits,整仓 gtest 304/0/0 + node 29/29 + 探针 15/15)。以下为移交装机阶段 / 后续计划 / 计划文档维护的跟进清单;Plan 系列①~⑥至此收官,无后续实施计划待编写,故本清单以装机项与文档回写为主。

## Important(0)

无。本分支无任何未闭环的实施侧缺陷;Plan 5 两条【必闭环】(I-1/I-2)均已在本分支落地并经独立评审+终审复核(见 final-review §2)。

## 装机 / 真机待办(3)

1. **I-1 KRC 侧干净停止触发器(遗留风险 4,装机实测裁决项)**:R2/R3 已使 PC 侧干净停止终局回 READY,但 "Stop S 配置为 MOVECORR break 条件" 仍未在真机配置与验证——这是 KRL EKI 循环阻塞期唯一的 PC 侧会话终结路径,未配置则操作员只能从示教器停止。装机时按 `kuka/rsi/ROS_RSI_CONTEXT.notes.md` 第 3 条 "COMMISSIONING DECISION (Plan 6 / followup I-1)" 增补项裁决,并回填 `docs/commissioning_checklist.md` Stage 2。
2. **人工浏览器冒烟 a~j(含 e2)待真机/真浏览器执行**:清单已收录于 `ros_ws/src/soft_robot_web_interface/README.md` "## Sim smoke checklist (plan Task 10)"。Task 10 已完成其可脚本化投影(探针 15/15:http 200、wrench 流、start/stop/zero 门控、标定全程 1365 帧 feedback、PREEMPTED 取消路径),但徽章渲染、曲线视觉、按钮门控镜像、断桥重连 UI(i 项)、Minor 7 启发式提示(e2 项)等浏览器交互面未经真实浏览器验证(计划决策 4:离线环境不硬套无头浏览器,遗留风险 7)。操作员首次部署时按 README 附录逐项执行。
3. **stop 宽限呈现窗 10 s 常数未经真机标定(遗留风险 6)**:`state_model.js` `STOP_GRACE_MS` 为 UI 呈现层启发式,方向安全(至多把真故障以 info 呈现 10 s,状态机本体不受影响);真机 5 拍闩锁 + 心跳恢复时序实测后可校准该常数。

## 部署环境注意(2)

4. **9090 端口冲突**:dev 机 9090 被 `mihomo` 代理常驻占用(Task 9/10 评审两次确认,非 ROS 残留);`web.launch` 默认 rosbridge_port=9090(rosbridge 上游默认,决策 3 锚定不改)。凡本机有代理/端口占用的环境,冒烟与部署用 `roslaunch soft_robot_web_interface web.launch rosbridge_port:=9091` + 探针 `ROSBRIDGE_PORT=9091`;另注意 curl 验证需 `--noproxy '*'`。
5. **rosbridge 0.11 安全面(遗留风险 5)**:无鉴权、明文 WS;部署限定隔离设备网段(README 已注明),TLS/auth 未做。若未来需跨网段访问,先做 C++ WebSocket 后端替换(规格 §5.7 工业部署选项)或反代加 TLS。

## 计划文本勘误回写清单(9,维护性,改 plan6 计划文档时顺手;全部为计划侧笔误/估算错,各任务报告+评审已裁决,无实施侧残留)

6. Task 2 Step 8:"runtime 19/19(17+2)" → **18/18(16+2)**(基数 16 经 stash 基线 grep+实跑双证;计划同段 "33+6=39" 自洽佐证)。
7. Task 3 Step 9:"runtime 19+2=21" → **18+2=20**(与上条同源笔误顺延;包级 41、整仓 304 推导不受影响)。
8. Task 4 Step 6:括注把矛盾措辞误记为 "clean READY" 字面,实际旧文为 "clean stop" / "ends the loop cleanly" / "record the observed manager behavior (READY/DEGRADED)" 三处;grep -c "clean READY"=0 断言本身保留(守门有效)。
9. Task 5 Step 4:RED 期望 "8 fail" → node:test 对 ERR_MODULE_NOT_FOUND 按**文件级计 1 fail**(runner 固有口径;Task 6/7 RED 同口径,已裁决先例)。
10. Task 9 Step 3 逐字稿第 25 行:离线 deb 束 **5 项 → 4 项**(删 `ros-noetic-rosapi-msgs`——0.11.17 时代 rosapi 未拆分该包,apt 无候选,评审独立复现)。
11. Task 10 E-A:探针标定等待 **60 s → 180 s**、硬帽 **90 s → 240 s**(标定固有 ~137 s:510° 行程 @5 dps + 收敛尾 + settle,源推导 ≈145 s 与实测吻合;240 s 留 1.75× 裕量)。
12. Task 10 E-B:`zero_sensor in READY` 检查**移至标定段之后**(计划字面顺序先置零→恒力 fz=5 流归 0→check 15 `bias_fz≈5` 永假,计划内部矛盾);15 checks 名称/断言/数量全保留。同时回写 bringup README 3f 期望的适用条件(**未清零流**)。
13. Task 10 R-1:探针头注释 "nine EKI PTP round-trips" → **"nine goal-mode orientation moves (RKorr via the RSI channel)"**(EKI mock 无 PTP 命令面;137 s 固有结论不受影响)。
14. (程序性备案)Task 3 简报"变更面"一节漏列 rsi/eki 两个注释级文件(计划 File Structure 已明列,以计划为准,非偏差);Task 8 实施者使用只读 git 查询自证变更面,评审裁决备案通过——两条均为流程记录,无回写动作。

## 已接受的遗留风险(本计划明示范围外,无行动项)

- **Web jog 未做**(风险 1):若未来做,先量化 rosbridge JSON 通道往返延迟并评估 stream_timeout 余量。
- **参数页只读**(风险 2):在线编辑需 manager 侧校验服务(规格 §14),v1 无。
- **correction values 无真实回显**(风险 3,规格冲突 2):以 saturation_count + mode/profile 代理;真实回显需 hw 增发布器(新增 C++)。

## 执行沉淀(约定,已在台账)

- 整仓 gtest 基线更新:**304**(52+59+8+61+44+39+41);node 单测 **29** 单列不并入;探针 15 checks。
- 冒烟 kill 纪律补强:不经 subshell 直启 + `kill $!` / roslaunch 本体 PID `kill -INT`;残留判定用 `ss`+`ps -p`(`pgrep -f` 会自命中 wrapper shell)。
