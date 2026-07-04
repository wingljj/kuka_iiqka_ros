# test/test_scenarios.py
from kuka_mujoco_sim.scenarios import ScenarioResult, format_report


def test_scenario_result_pass_aggregates():
    r = ScenarioResult('demo')
    r.check('a', True, 'ok')
    r.check('b', True, 'ok')
    assert r.passed is True


def test_scenario_result_fail_if_any_check_fails():
    r = ScenarioResult('demo')
    r.check('a', True, 'ok')
    r.check('b', False, 'net force 0.9 > 0.5')
    assert r.passed is False


def test_format_report_contains_pass_fail_and_names():
    r1 = ScenarioResult('free_space'); r1.check('x', True, '')
    r2 = ScenarioResult('drift'); r2.check('y', False, 'no drift')
    text = format_report([r1, r2])
    assert 'free_space' in text and 'drift' in text
    assert 'PASS' in text and 'FAIL' in text


def test_format_report_overall_fail_when_any_fail():
    r1 = ScenarioResult('a'); r1.check('x', True, '')
    r2 = ScenarioResult('b'); r2.check('y', False, '')
    text = format_report([r1, r2])
    assert 'OVERALL: FAIL' in text
