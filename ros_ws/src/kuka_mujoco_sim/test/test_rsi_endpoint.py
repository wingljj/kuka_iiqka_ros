from kuka_mujoco_sim.rsi_endpoint import RsiEndpoint
from kuka_mujoco_sim.rsi_codec import build_rob_frame

class FakeSock:
    def __init__(self): self.sent = []
    def sendto(self, data, addr): self.sent.append((data, addr))

def test_send_state_emits_rob_and_tracks_ipoc():
    ep = RsiEndpoint(ipoc_start=1000, ipoc_step=4)
    ep.set_initial_target((0, 0, 800, 0, 0, 0))
    sock = FakeSock()
    ep.send_state(sock, (1, 2, 800, 0, 0, 0))
    assert len(sock.sent) == 1
    assert b'<Rob' in sock.sent[0][0]
    assert ep.awaited_ipoc == 1000

def test_apply_reply_integrates_rkorr():
    ep = RsiEndpoint(ipoc_start=1000, ipoc_step=4)
    ep.set_initial_target((0, 0, 800, 0, 0, 0))
    sock = FakeSock()
    ep.send_state(sock, (0, 0, 800, 0, 0, 0))  # awaited=1000
    sen = (b'<Sen Type="ROS"><RKorr X="0.1" Y="0" Z="-0.2" A="0" B="0" C="0"/>'
           b'<Stop S="0"/><Watchdog W="1"/><IPOC>1000</IPOC></Sen>')
    new_target = ep.apply_reply(sen)
    assert new_target is not None
    assert abs(new_target[0] - 0.1) < 1e-9
    assert abs(new_target[2] - 799.8) < 1e-9

def test_apply_reply_rejects_wrong_ipoc():
    ep = RsiEndpoint(ipoc_start=1000, ipoc_step=4)
    ep.set_initial_target((0, 0, 800, 0, 0, 0))
    sock = FakeSock()
    ep.send_state(sock, (0, 0, 800, 0, 0, 0))  # awaited=1000
    sen = (b'<Sen><RKorr X="0.1" Y="0" Z="0" A="0" B="0" C="0"/>'
           b'<IPOC>999</IPOC></Sen>')
    assert ep.apply_reply(sen) is None

def test_ipoc_increments_each_state():
    ep = RsiEndpoint(ipoc_start=1000, ipoc_step=4)
    ep.set_initial_target((0, 0, 800, 0, 0, 0))
    sock = FakeSock()
    ep.send_state(sock, (0, 0, 800, 0, 0, 0))
    assert ep.awaited_ipoc == 1000
    ep.send_state(sock, (0, 0, 800, 0, 0, 0))
    assert ep.awaited_ipoc == 1004
