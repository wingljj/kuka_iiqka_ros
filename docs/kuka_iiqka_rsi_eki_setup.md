# KUKA iiQKA RSI/EKI Setup & Field-by-Field Checklist

Templates under `kuka/` are text deliverables (plan decision 17): they
cannot be executed or unit-tested on the ROS host, so THIS checklist is
the acceptance artifact. Work through every `- [ ]` item on the real
cell, against the local `ref/` manuals and the running ROS side.

Schema authorities (do not "fix" the templates against anything else):

- EKI channel: `ros_ws/src/kuka_eki_bridge/include/kuka_eki_bridge/eki_frame.h`
- RSI frames: `ros_ws/src/kuka_rsi_hw_interface/include/kuka_rsi_hw_interface/rsi_frame.h`
- Ports: `ros_ws/src/kuka_eki_bridge/config/kuka_eki.yaml` (EKI TCP 54600),
  `ros_ws/src/kuka_rsi_hw_interface/config/kuka_rsi.yaml` (RSI UDP 49152)

## 1. Prerequisites
- [ ] KR C5, iiQKA.OS2 >= 9.2; iiQKA.RobotSensorInterface 6.2;
      iiQKA.EthernetKRL 6.1 (spec section 6 floors).
- [ ] Network plan: KRC <-> ROS host on one subnet; RSI UDP 49152
      (kuka_rsi.yaml), EKI TCP 54600 (kuka_eki.yaml).

## 2. KRL syntax verification (do FIRST, against ref/ manuals)
- [ ] Every EKI_* call in ROS_RSI_SERVO.SRC matches the EthernetKRL 6.1
      manual signature (EKI_Load/Open/CheckBuffer/ReadNext/GetInt/GetReal/
      SetInt/SetReal/Send/Close/Unload). Fix syntax here, NOT the schema.
- [ ] Every RSI_* call matches the RobotSensorInterface 6.2 manual
      (RSI_LOAD/ACTIVATE/PROCESS_ON/MOVECORR/PROCESS_OFF/DEACTIVATE/
      RSI_UNLOAD) including the PROCESS_ON relative-mode argument.
- [ ] B_TO_I helper: replace with the manual-blessed BOOL->INT idiom if
      unavailable on the target KSS.
- [ ] $TIMER usage and timer number ROS_HB_TIMER_NO free on this cell.

## 3. EkiConfig field-by-field (vs eki_frame.h — the single authority)
For EACH row confirm XML path, attribute name (case!), and type:
- [ ] RECEIVE RobotCommand/Cmd @Seq @Action @Value (INT x3)
- [ ] RECEIVE RobotCommand/Tool @X @Y @Z @A @B @C (REAL x6)
- [ ] RECEIVE RobotCommand/Base @X @Y @Z @A @B @C (REAL x6)
- [ ] SEND RobotState/Ack @Seq @Ok @Code (INT x3)
- [ ] SEND RobotState/Prog @Ready @RsiActive @Fault @Mode (INT x4)
- [ ] SEND RobotState/Tool @X @Y @Z @A @B @C (REAL x6)
- [ ] Action codes in the SRC SWITCH: 0..6 in eki_frame.h order.
- [ ] Error codes: 0 OK / 1 NOT_READY / 2 FAULTED only.
- [ ] EXTERNAL TYPE=Client, INTERNAL PORT=54600/TCP (decision 5).
- [ ] Heartbeat: Ack.Seq=0 every 100 ms while the program runs
      (bridge freshness threshold 1 s; manager threshold 5 s).

## 4. RSI context (vs rsi_frame.h)
- [ ] SEND: RIst(X..C), AIPos(A1..A6), Delay(D), Mode(M), IPOC.
- [ ] RECEIVE: RKorr(X..C), Stop(S), Watchdog(W), IPOC echo.
- [ ] SENTYPE="ROS"; PORT=49152; ROS host IP set.
- [ ] POSCORR: RELATIVE mode, coordinate system BASE, clamp limits set.
- [ ] Timeout behavior: zero correction + stop after missed answers.

## 5. First-contact smoke (with the ROS side up, robot in T1)
- [ ] eki: rostopic echo /kuka/eki/state -> connected+state_fresh true,
      heartbeat age < 0.2 s.
- [ ] rosservice call /kuka/eki/get_tool returns the pendant's $TOOL.
- [ ] START_RSI from the manager; /kuka/rsi/state connected: True,
      zero-output loop stable (commissioning checklist stage 2).
