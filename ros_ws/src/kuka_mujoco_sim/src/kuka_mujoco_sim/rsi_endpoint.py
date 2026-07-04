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
        if not self.awaiting or parsed['ipoc'] != self.awaited_ipoc:
            self.stats['ipoc_errors'] += 1
            return None
        self.awaiting = False
        rk = parsed['rkorr']
        self.pose_target6 = tuple(
            self.pose_target6[i] + rk[i] for i in range(6))
        return self.pose_target6
