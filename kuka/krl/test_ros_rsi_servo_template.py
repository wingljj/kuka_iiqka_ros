#!/usr/bin/env python3
"""Static contract checks for the ROS_RSI_SERVO KRL template.

The ROS host cannot compile KRL, so these tests pin the protocol/lifecycle
contract that the text template must preserve.
"""

from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "kuka" / "krl" / "ROS_RSI_SERVO.SRC"
DAT = ROOT / "kuka" / "krl" / "ROS_RSI_SERVO.DAT"
DOC = ROOT / "docs" / "kuka_iiqka_rsi_eki_setup.md"
RSI_NOTES = ROOT / "kuka" / "rsi" / "ROS_RSI_CONTEXT.notes.md"
RSIX = ROOT / "kuka" / "rsi" / "ROS_RSI_CONTEXT.rsix"


def text(path):
    return path.read_text(encoding="utf-8")


class RosRsiServoTemplateContract(unittest.TestCase):
    def test_protocol_schema_and_action_codes_stay_fixed(self):
        src = text(SRC)
        self.assertIn("0 QUERY_STATE  1 START_RSI  2 STOP_RSI  3 SET_MODE", src)
        self.assertIn("4 RESET_FAULT  5 GET_TOOL   6 SET_TOOL_BASE", src)
        for case in range(7):
            self.assertRegex(src, rf"\bCASE {case}\b")
        for field in (
            "RobotState/Ack@Seq",
            "RobotState/Ack@Ok",
            "RobotState/Ack@Code",
            "RobotState/Prog@Ready",
            "RobotState/Prog@RsiActive",
            "RobotState/Prog@Fault",
            "RobotState/Prog@Mode",
            "RobotState/Tool@X",
            "RobotState/Tool@Y",
            "RobotState/Tool@Z",
            "RobotState/Tool@A",
            "RobotState/Tool@B",
            "RobotState/Tool@C",
        ):
            self.assertIn(field, src)

    def test_heartbeat_timer_is_explicitly_initialized_and_started(self):
        src = text(SRC)
        dat = text(DAT)
        self.assertIn("ROS_HB_TIMER_NO", dat)
        self.assertIn("$TIMER_STOP[ROS_HB_TIMER_NO] = TRUE", src)
        self.assertIn("$TIMER[ROS_HB_TIMER_NO] = 0", src)
        self.assertIn("$TIMER_STOP[ROS_HB_TIMER_NO] = FALSE", src)
        self.assertRegex(src, r"ROS_SEND_STATE\(0, 1, (0|ROS_ERR_OK)\)")

    def test_blocking_movecorr_lifecycle_is_documented(self):
        src = text(SRC)
        docs = text(DOC) + "\n" + text(RSI_NOTES)
        self.assertIn("RSI_MOVECORR() blocks", src)
        self.assertIn("STOP_RSI cannot be serviced while RSI_MOVECORR() is active", src)
        self.assertRegex(docs, r"Stop S|<Stop S")
        self.assertIn("STOP_RSI", docs)
        self.assertIn("heartbeat", docs.lower())

    def test_critical_calls_are_checked_and_fault_latched(self):
        src = text(SRC)
        self.assertIn("ROS_EKI_OK", text(DAT))
        self.assertIn("ROS_CHECK_RET", src)
        self.assertIn("ROS_LATCH_FAULT", src)
        for call in (
            "EKI_Load",
            "EKI_Open",
            "EKI_ReadNext",
            "EKI_GetInt",
            "EKI_Send",
            "RSI_LOAD",
            "RSI_ACTIVATE",
            "RSI_PROCESS_ON",
            "RSI_PROCESS_OFF",
            "RSI_DEACTIVATE",
            "RSI_UNLOAD",
        ):
            self.assertRegex(src, rf"RET = {call}\(")
        self.assertNotIn("B_TO_I", src)

    def test_set_tool_base_is_rejected_while_active_and_atomic(self):
        src = text(SRC)
        self.assertRegex(
            src,
            r"CASE 6[\s\S]*IF bRsiActive THEN[\s\S]*ROS_SEND_STATE\(nSeq, 0, (1|ROS_ERR_NOT_READY)\)",
        )
        self.assertIn("DECL FRAME fNewTool", src)
        self.assertIn("DECL FRAME fNewBase", src)
        self.assertLess(src.index("fNewTool.X"), src.index("$TOOL = fNewTool"))
        self.assertLess(src.index("fNewBase.X"), src.index("$BASE = fNewBase"))
        self.assertIn("$TOOL = fNewTool", src)
        self.assertIn("$BASE = fNewBase", src)

    def test_rsix_context_template_matches_rsi_contract(self):
        self.assertTrue(RSIX.exists(), "ROS_RSI_CONTEXT.rsix must be present")
        root = ET.parse(RSIX).getroot()
        self.assertTrue(root.tag.endswith("RsiContext"))
        xml = text(RSIX)
        self.assertLess(xml.index("<BlueprintCollections"), xml.index("<Constructs"))
        self.assertIn('ObjType="Ethernet"', xml)
        self.assertIn('ParamValue="ROS_RSI_ETHERNET.xml"', xml)
        self.assertIn('ObjType="PosCorr"', xml)
        self.assertIn('ObjType="Stop"', xml)
        self.assertIn('ParamValue="4"', xml)  # Stop Mode = ExitMoveCorr
        for signal in (
            "RKorr.X",
            "RKorr.Y",
            "RKorr.Z",
            "RKorr.A",
            "RKorr.B",
            "RKorr.C",
            "Stop.S",
            "Watchdog.W",
        ):
            self.assertIn(signal, xml)
        self.assertRegex(xml, r'ObjId="Limit_[XYZABC]"')
        self.assertIn('Name="CorrectionCoordinateSystem"', xml)
        self.assertIn('ParamValue="BASE"', xml)


if __name__ == "__main__":
    unittest.main()
