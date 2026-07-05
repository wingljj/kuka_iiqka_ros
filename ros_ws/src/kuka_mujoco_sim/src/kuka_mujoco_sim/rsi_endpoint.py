"""RSI UDP client endpoint: sends <Rob> state (actual pose), receives <Sen>
command (RKorr), integrates RKorr into the target pose. Mirrors RsiMockCore
but the 'actual pose' is supplied by the physics world, not integrated here.
"""
from .rsi_codec import build_rob_frame, parse_sen_frame


class RsiEndpoint:
    def __init__(self, target_ip='127.0.0.1', target_port=49152,
                 ipoc_start=1000, ipoc_step=4):
        self.addr = (target_ip, target_port)
        self._next_ipoc = ipoc_start
        self._step = ipoc_step
        self.awaited_ipoc = None
        self.awaiting = False
        # Highest IPOC whose RKorr we have already integrated. The hw_interface
        # runs its OWN 250 Hz read/write loop, decoupled from ours, so the
        # reply sitting in our socket at recv time echoes an EARLIER frame than
        # the one we just sent (a persistent one-frame skew). Requiring an exact
        # awaited_ipoc match therefore rejected ~95% of replies and RKorr never
        # accumulated -> the arm looked dead under force. RKorr is a per-cycle
        # increment, so the correct rule is monotonic: apply any reply whose
        # IPOC is newer than the last one applied (dropping only true duplicates
        # / out-of-order stragglers). This restores the force->motion loop.
        self._last_applied_ipoc = -1
        self.pose_target6 = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.stats = {'sent': 0, 'replies': 0, 'ipoc_errors': 0, 'parse_errors': 0}

    def set_initial_target(self, pose6):
        self.pose_target6 = tuple(float(v) for v in pose6)

    def send_state(self, sock, actual_pose6):
        frame = build_rob_frame(actual_pose6, self._next_ipoc)
        sock.sendto(frame, self.addr)
        self.awaited_ipoc = self._next_ipoc
        self.awaiting = True
        self._next_ipoc += self._step
        self.stats['sent'] += 1

    def apply_reply(self, data):
        parsed = parse_sen_frame(data)
        if parsed is None:
            self.stats['parse_errors'] += 1
            return None
        self.stats['replies'] += 1
        # Monotonic acceptance: integrate any reply newer than the last applied
        # (see _last_applied_ipoc note). Duplicates / stragglers with an IPOC we
        # have already consumed are dropped so no RKorr is applied twice.
        ipoc = parsed['ipoc']
        if ipoc <= self._last_applied_ipoc:
            self.stats['ipoc_errors'] += 1
            return None
        self._last_applied_ipoc = ipoc
        self.awaiting = False
        rk = parsed['rkorr']
        self.pose_target6 = tuple(
            self.pose_target6[i] + rk[i] for i in range(6))
        return self.pose_target6
