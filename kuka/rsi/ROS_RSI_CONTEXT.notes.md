# ROS_RSI_CONTEXT — signal-flow notes (build in RSI Visual / iiQKA tooling)

The `ROS_RSI_CONTEXT.rsix` template is hand-authored and should be opened
and re-saved in RSIVisual on the target controller/toolchain before
deployment. This note pins what the generated context must contain
(checklist section 4):

1. ETHERNET object using ROS_RSI_ETHERNET.xml (4 ms cycle, timeout
   behavior = stop with zero correction after N missed answers).
2. RKorr X/Y/Z/A/B/C inputs -> per-axis clamp blocks (KUKA-side hard
   limit, spec 12.3; ROS applies its own limits before sending) ->
   POSCORR object, mode RELATIVE, correction coordinate system BASE
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
4. Watchdog W input: monotonically increasing PC liveness counter;
   wire to a timeout monitor if the deployment wants PC-side liveness
   enforced on the KRC too.
5. POSCORR limits: configure the maximum overall correction envelope
   (mm/deg) allowed by the cell safety concept.
