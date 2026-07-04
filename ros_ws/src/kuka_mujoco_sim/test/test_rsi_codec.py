from kuka_mujoco_sim.rsi_codec import build_rob_frame, parse_sen_frame


def test_build_rob_frame_contains_fields():
    b = build_rob_frame((10.0, 20.0, 800.0, 1.0, 2.0, 3.0), 1000)
    s = b.decode('ascii')
    assert '<Rob Type="KUKA">' in s
    assert 'X="10.0000"' in s and 'Z="800.0000"' in s
    assert 'A="1.0000"' in s and 'C="3.0000"' in s
    assert '<IPOC>1000</IPOC>' in s
    assert '<Mode M="1"/>' in s
    assert 'AIPos' in s


def test_parse_sen_frame_roundtrip():
    sen = (b'<Sen Type="ROS">'
           b'<RKorr X="0.1000" Y="-0.2000" Z="0.3000" A="0.0100" B="0.0200" C="-0.0300"/>'
           b'<Stop S="0"/><Watchdog W="5"/><IPOC>1000</IPOC></Sen>')
    out = parse_sen_frame(sen)
    assert out is not None
    assert abs(out['rkorr'][0] - 0.1) < 1e-9
    assert abs(out['rkorr'][1] + 0.2) < 1e-9
    assert abs(out['rkorr'][5] + 0.03) < 1e-9
    assert out['stop'] == 0
    assert out['ipoc'] == 1000


def test_parse_sen_frame_missing_rkorr():
    assert parse_sen_frame(b'<Sen><IPOC>1</IPOC></Sen>') is None


def test_parse_sen_frame_garbage():
    assert parse_sen_frame(b'not xml') is None


def test_build_then_parse_via_rob_is_not_sen():
    # build_rob_frame output is <Rob>, parse_sen_frame must reject it.
    b = build_rob_frame((0, 0, 800, 0, 0, 0), 1000)
    assert parse_sen_frame(b) is None
