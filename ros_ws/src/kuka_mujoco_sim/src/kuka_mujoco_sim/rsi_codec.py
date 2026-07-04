"""RSI <Rob>/<Sen> wire codec, matching the C++ rsi_frame.cpp format.

We emit <Rob> state frames (we impersonate the KRC) and parse <Sen> command
frames (from kuka_rsi_hw_interface). Uses ElementTree for parsing.
"""
import xml.etree.ElementTree as ET


def build_rob_frame(pose6, ipoc, axes6=None):
    """pose6 = (X, Y, Z, A, B, C) in mm/deg. Returns ASCII bytes."""
    x, y, z, a, b, c = pose6
    if axes6 is None:
        axes6 = (0.0,) * 6
    a1, a2, a3, a4, a5, a6 = axes6
    s = (
        '<Rob Type="KUKA">'
        '<RIst X="{:.4f}" Y="{:.4f}" Z="{:.4f}" A="{:.4f}" B="{:.4f}" C="{:.4f}"/>'
        '<AIPos A1="{:.4f}" A2="{:.4f}" A3="{:.4f}" A4="{:.4f}" A5="{:.4f}" A6="{:.4f}"/>'
        '<Delay D="0"/>'
        '<Mode M="1"/>'
        '<IPOC>{}</IPOC>'
        '</Rob>'
    ).format(x, y, z, a, b, c, a1, a2, a3, a4, a5, a6, int(ipoc))
    return s.encode('ascii')


def parse_sen_frame(data):
    """Parse a <Sen> command frame. Returns dict or None on any failure."""
    try:
        root = ET.fromstring(data)
    except ET.ParseError:
        return None
    if root.tag != 'Sen':
        return None
    rkorr = root.find('RKorr')
    ipoc = root.find('IPOC')
    if rkorr is None or ipoc is None or ipoc.text is None:
        return None
    try:
        vals = tuple(float(rkorr.attrib[k]) for k in ('X', 'Y', 'Z', 'A', 'B', 'C'))
        ipoc_val = int(ipoc.text.strip())
        stop_el = root.find('Stop')
        stop = int(stop_el.attrib.get('S', '0')) if stop_el is not None else 0
    except (KeyError, ValueError):
        return None
    return {'rkorr': vals, 'stop': stop, 'ipoc': ipoc_val}
