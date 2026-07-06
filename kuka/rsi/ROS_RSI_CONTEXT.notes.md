# ROS_RSI_CONTEXT — signal-flow notes (build in RSI Visual / iiQKA tooling)

The `ROS_RSI_CONTEXT.rsix` file is derived from the generated
iiQKA OS2 RSIVisual 6.2 example `rsi_joint_pos.rsix` in KUKA's
`kroshu/kuka-external-control-sdk` repository, not from a minimal
hand-written schema. It keeps the complete generated
`BlueprintCollections` block because RSIVisual import depends on that
metadata. Open and re-save it in RSIVisual on the target
controller/toolchain before deployment. This note pins what the context
must contain (checklist section 4):

1. ETHERNET object using ROS_RSI_ETHERNET.xml (4 ms cycle, timeout
   behavior = stop with zero correction after N missed answers).
2. RKorr X/Y/Z/A/B/C inputs -> per-axis clamp blocks (KUKA-side hard
   limit, spec 12.3; ROS applies its own limits before sending) ->
   PosCorr object, mode RELATIVE, `RefCorrSys=Base`
   (spec 6.1 default; TOOL stays a configurable experiment).
3. Stop S input -> stop/brake path (PC-requested stop, e.g. latched
   fault in the hw interface sends `<Stop S="1"/>` in the real-time
   RSI `<Sen>` reply).
   COMMISSIONING DECISION (Plan 6 / followup I-1): wiring Stop S as a
   MOVECORR break condition is the only PC-side path that can end an
   RSI-active phase while the KRL EKI loop is blocked; without it the
   operator must stop from the pendant. Verify on the real cell and
   record the choice in the commissioning checklist Stage 2.
   EKI STOP_RSI is only reachable after RSI_MOVECORR() returns to the
   management loop, so it is cleanup, not the real-time motion stop.
4. Watchdog W input: monotonically increasing PC liveness counter.
   The `.rsix` reserves ETHERNET Out8 for this signal but does not invent
   a generic monitor block. Wire it in RSIVisual only after selecting the
   real cell's desired KRC-side watchdog behavior.
5. POSCORR limits: configure the maximum overall correction envelope
   (mm/deg) allowed by the cell safety concept.
