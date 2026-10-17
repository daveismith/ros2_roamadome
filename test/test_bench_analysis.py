# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for ros2_roamadome.bench.analysis (no ROS, no hardware)."""

import json
import math
import os
import subprocess
import sys

import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from ros2_roamadome.bench import analysis  # noqa: E402

REPORT_DT = 0.028   # firmware #DPREPORT28
CYCLE = 1.0 / 30.0  # controller manager period: the host stamp resolution
T0 = 1792023000.0


def host_stamp(t_fw):
    """Stamp of the controller cycle that reads a line sent at firmware time t_fw."""
    return math.ceil(t_fw / CYCLE - 1e-9) * CYCLE


# The plugin stamps each line as it publishes it, so two lines read in one cycle are
# a few tens of microseconds apart rather than exactly equal (measured on the robot).
LINE_JITTER_S = 6e-5


def make_step(index, percent, speed_deg_s, start_t, start_deg, settle_s, measure_s,
              angle_sign=-1, gap=None):
    """
    Synthesise one step's rows like the plugin would publish them.

    The angle moves at angle_sign * sign(percent) * speed_deg_s, is quantised to
    integer degrees, reported every 28 ms and stamped at the 33 ms cycle (so some
    pairs share a stamp). gap=(start_s, length_s) drops the stamps of a periodic
    config refresh: lines in the gap get the stamp of the first cycle after it.
    """
    direction = 1 if percent > 0 else -1
    rate = angle_sign * direction * speed_deg_s
    rows = []
    n = int((settle_s + measure_s) / REPORT_DT)
    for k in range(n):
        t_fw = k * REPORT_DT
        stamp = host_stamp(t_fw)
        if gap and gap[0] <= stamp < gap[0] + gap[1]:
            stamp = host_stamp(gap[0] + gap[1])
        angle = start_deg + rate * t_fw
        rows.append(analysis.Sample(
            t_host=start_t + stamp + (k % 2) * LINE_JITTER_S, seq=k,
            position_deg=int(round(angle)) % 360,
            unwrapped_rad=math.radians(angle), velocity_rad_s=float('nan'),
            commanded_percent=percent, direction=direction, step_index=index,
            phase=analysis.PHASE_SETTLE if t_fw < settle_s else analysis.PHASE_MEASURE))
    return rows


def replace_positions(rows, positions):
    return [r._replace(position_deg=p % 360) for r, p in zip(rows, positions)]


CONFIG = {
    'velocity_window_s': 0.5,
    'stall_deg_s': 3.0,
    'stall_s': 2.0,
    'max_plausible_deg_s': 720.0,
    'raw_motor': True,
}


# ---------------------------------------------------------------------------
# 7.1
# ---------------------------------------------------------------------------

def test_shortest_delta_and_unwrap_across_zero_both_ways():
    assert analysis.shortest_delta(359, 0) == 1
    assert analysis.shortest_delta(0, 359) == -1
    assert analysis.shortest_delta(10, 190) == -180
    assert analysis.unwrap_degrees([357, 358, 359, 0, 1, 2]) == [357, 358, 359, 360, 361, 362]
    assert analysis.unwrap_degrees([2, 1, 0, 359, 358]) == [2, 1, 0, -1, -2]


def test_regression_slope_exact_and_degenerate():
    times = [0.0, 0.1, 0.2, 0.3]
    assert analysis.regression_slope(times, [5 + 2 * t for t in times]) == pytest.approx(2.0)
    assert math.isnan(analysis.regression_slope([1.0, 1.0], [0.0, 5.0]))
    assert math.isnan(analysis.regression_slope([1.0], [0.0]))


def test_trailing_velocity_nan_until_window_populated_then_accurate():
    times = [i * 0.03 for i in range(60)]
    values = [100.0 * t for t in times]
    v = analysis.trailing_velocities(times, values, 0.5)
    # needs 4 samples spanning >= 0.2 s: the sample at 0.21 s is the first
    first = next(i for i, x in enumerate(v) if not math.isnan(x))
    assert times[first] >= 0.2 - 1e-9
    assert all(math.isnan(x) for x in v[:first])
    assert v[-1] == pytest.approx(100.0, rel=0.02)


def test_trailing_velocity_quantised_stream_within_five_percent():
    times = [i * REPORT_DT for i in range(100)]
    values = analysis.unwrap_degrees([int(round(30.0 * t)) % 360 for t in times])
    v = analysis.trailing_velocities(times, values, 0.5)
    assert v[-1] == pytest.approx(30.0, rel=0.05)


def test_mean_std_ignores_nan():
    mean, std = analysis.mean_std([1.0, float('nan'), 3.0])
    assert mean == pytest.approx(2.0)
    assert std == pytest.approx(math.sqrt(2.0))
    assert all(math.isnan(x) for x in analysis.mean_std([float('nan')]))


# ---------------------------------------------------------------------------
# 7.2
# ---------------------------------------------------------------------------

def test_stall_detector_needs_full_window_then_flags_standstill():
    det = analysis.StallDetector(3.0, 2.0)
    flags = [det.update(i * 0.1, 50.0) for i in range(25)]
    assert not any(flags[:20])   # samples span < 2 s
    assert flags[20] and flags[-1]


def test_stall_detector_moving_creeping_and_flicker():
    moving = analysis.StallDetector(3.0, 2.0)
    assert not any(moving.update(i * 0.03, 10.0 * i * 0.03) for i in range(200))  # 10 deg/s
    creep = analysis.StallDetector(3.0, 2.0)
    assert any(creep.update(i * 0.03, 2.0 * i * 0.03) for i in range(200))  # 2 deg/s
    flicker = analysis.StallDetector(3.0, 2.0)
    assert any(flicker.update(i * 0.03, float(i % 2)) for i in range(200))
    assert analysis.find_stall([0.0, 1.0, 2.5], [0.0, 0.0, 0.0], 3.0, 2.0) == 2
    assert analysis.find_stall([0.0, 1.0, 2.5], [0.0, 10.0, 20.0], 3.0, 2.0) is None


def test_jumps_skip_same_cycle_pairs_and_tolerate_gaps():
    times = [0.0, 0.033, 0.033, 0.066, 0.296, 0.329]
    degrees = [100, 103, 105, 108, 140, 143]   # 230 ms gap moving ~140 deg/s: plausible
    jumps, zero_dt, _ = analysis.score_jumps(times, degrees, 720.0)
    assert jumps == []
    assert zero_dt == 1
    # A large change with dt = 0 is not scored (no interval to judge it by)...
    jumps, zero_dt, _ = analysis.score_jumps([0.0, 0.0], [10, 90], 720.0)
    assert jumps == [] and zero_dt == 1
    # ...nor are normal moves of lines stamped 60 us apart in one cycle
    jumps, same_cycle, _ = analysis.score_jumps([0.0, 0.033, 0.03306, 0.066],
                                                [100, 104, 108, 112], 720.0)
    assert jumps == [] and same_cycle == 1
    assert not analysis.is_jump(4, 6e-5, 720.0)
    # ...but the return from a glitch is.
    jumps, _, idx = analysis.score_jumps([0.0, 0.033, 0.033, 0.066], [10, 11, 90, 12], 720.0,
                                         [20, 20, 20, 20])
    assert len(jumps) == 1 and idx == {3}
    assert jumps[0]['start_deg'] == 90 and jumps[0]['delta_deg'] == -78
    assert jumps[0]['commanded_percent'] == 20


def test_wrap_allowance_applies_only_to_steps_across_zero():
    # 355 -> 17 at 6.65 rad/s (the dead zone the controller skips near 0): 22 deg in
    # 33 ms against a 576 deg/s threshold is a jump without the allowance.
    times = [0.0, 0.0333, 0.0666, 0.0999]
    jumps, _, _ = analysis.score_jumps(times, [341, 355, 17, 29], 576.02)
    assert len(jumps) == 1 and jumps[0]['start_deg'] == 355
    jumps, _, _ = analysis.score_jumps(times, [341, 355, 17, 29], 576.02,
                                       wrap_allowance_deg=10.0)
    assert jumps == []
    # The same 22 deg step away from 0 is still a jump with the allowance.
    jumps, _, _ = analysis.score_jumps(times, [100, 110, 132, 142], 576.02,
                                       wrap_allowance_deg=10.0)
    assert len(jumps) == 1 and jumps[0]['start_deg'] == 110
    # Downwards across 0 too.
    jumps, _, _ = analysis.score_jumps(times, [29, 17, 355, 341], 576.02,
                                       wrap_allowance_deg=10.0)
    assert jumps == []
    assert analysis.crosses_zero(355, 22) and analysis.crosses_zero(17, -22)
    assert not analysis.crosses_zero(100, 22) and not analysis.crosses_zero(0, 5)


def test_jump_threshold_boundary():
    # threshold = 720 * 0.033 + 2 = 25.76
    assert not analysis.is_jump(25, 0.033, 720.0)
    assert analysis.is_jump(26, 0.033, 720.0)
    assert analysis.is_jump(-26, 0.033, 720.0)
    assert not analysis.is_jump(100, 0.0, 720.0)


def test_reversals_need_two_consecutive_backward_pairs():
    forward = [100, 98, 96, 94]
    assert analysis.score_reversals(forward, -1) == (0, 0)
    one_back = [100, 98, 101, 99, 97]
    assert analysis.score_reversals(one_back, -1) == (0, 1)
    two_back = [100, 98, 101, 104, 102, 100]
    assert analysis.score_reversals(two_back, -1) == (1, 2)
    long_back = [100, 103, 106, 109, 112]   # one run of 4 backward pairs: one event
    assert analysis.score_reversals(long_back, -1) == (1, 4)
    small = [100, 101, 102, 103]            # |d| < 2 never counts
    assert analysis.score_reversals(small, -1) == (0, 0)
    assert analysis.score_reversals(two_back, -1, exclude={3}) == (0, 1)
    assert analysis.score_reversals(two_back, 0) == (0, 0)


def test_jump_histogram_bins():
    jumps = [{'start_deg': 0}, {'start_deg': 9}, {'start_deg': 10}, {'start_deg': 359}]
    counts = analysis.jump_histogram(jumps)
    assert len(counts) == 36
    assert counts[0] == 2 and counts[1] == 1 and counts[35] == 1


# ---------------------------------------------------------------------------
# 7.3 helpers
# ---------------------------------------------------------------------------

def test_measure_duration_rule():
    assert analysis.measure_duration(180.0, 4.0, 30.0) == pytest.approx(4.0)
    assert analysis.measure_duration(36.0, 4.0, 30.0) == pytest.approx(10.0)
    assert analysis.measure_duration(5.0, 4.0, 30.0) == pytest.approx(30.0)
    assert analysis.measure_duration(float('nan'), 4.0, 30.0) == pytest.approx(4.0)
    assert analysis.measure_duration(-36.0, 4.0, 30.0) == pytest.approx(10.0)


def test_fit_affine_exact():
    ps = [20, 30, 50, 100]
    fit = analysis.fit_affine(ps, [0.03 * (p - 12) for p in ps])
    assert fit['k'] == pytest.approx(0.03)
    assert fit['p0'] == pytest.approx(12.0)
    assert fit['r_squared'] == pytest.approx(1.0)
    assert analysis.fit_affine([20, 20], [1.0, 1.1]) is None
    assert analysis.fit_affine([20], [1.0]) is None


def test_fit_affine_matches_numpy_when_available():
    np = pytest.importorskip('numpy')
    ps = [15, 20, 25, 30, 40, 50, 60, 80, 100]
    speeds = [0.1, 0.27, 0.41, 0.55, 0.86, 1.13, 1.45, 1.98, 2.61]
    fit = analysis.fit_affine(ps, speeds)
    slope, intercept = np.polyfit(ps, speeds, 1)
    assert fit['k'] == pytest.approx(slope)
    assert fit['p0'] == pytest.approx(-intercept / slope)
    pred = np.polyval([slope, intercept], ps)
    r2 = 1 - np.sum((np.array(speeds) - pred) ** 2) / np.sum(
        (np.array(speeds) - np.mean(speeds)) ** 2)
    assert fit['r_squared'] == pytest.approx(r2)


def test_min_percent_bracket_and_binary_search_converge():
    assert analysis.min_percent_bracket([]) == (None, None)
    assert analysis.min_percent_bracket([(20, True), (30, True)]) == (None, 20)
    # a stall above the lowest moving percent does not bracket
    assert analysis.min_percent_bracket([(10, False), (20, True), (40, False)]) == (10, 20)
    for threshold in (11, 16, 17, 19, 20, 23):
        entries = [(p, p >= threshold) for p in (10, 15, 20, 25, 30)]
        probes = []
        while True:
            probe = analysis.next_search_probe(entries)
            if probe is None:
                break
            probes.append(probe)
            entries.append((probe, probe >= threshold))
        low, high = analysis.min_percent_bracket(entries)
        assert high - low <= 2
        assert low < threshold <= high
        assert len(probes) <= 3


# ---------------------------------------------------------------------------
# CSV and JSON
# ---------------------------------------------------------------------------

def test_csv_round_trip_is_exact_and_tolerates_truncation(tmp_path):
    rows = make_step(0, 20, 86.0, T0, 10, 0.5, 0.5)
    rows[3] = rows[3]._replace(velocity_rad_s=1.234567890123)
    path = str(tmp_path / 'samples.csv')
    analysis.write_csv(path, rows)
    back = analysis.read_csv(path)
    assert len(back) == len(rows)
    for a, b in zip(rows, back):
        assert a.t_host == b.t_host and a.position_deg == b.position_deg
        assert a.phase == b.phase and a.step_index == b.step_index
        assert (math.isnan(a.velocity_rad_s) and math.isnan(b.velocity_rad_s)) or \
            a.velocity_rad_s == b.velocity_rad_s
    with open(path) as f:
        header = f.readline().strip()
    assert header == ','.join(analysis.CSV_FIELDS)
    with open(path, 'a') as f:
        f.write('1792023000.5,7,12')   # a partial line, as after SIGKILL
    assert len(analysis.read_csv(path)) == len(rows)
    bad = tmp_path / 'bad.csv'
    bad.write_text('t_host,seq\n1,2\n')
    with pytest.raises(ValueError):
        analysis.read_csv(str(bad))


def test_dumps_json_is_strict():
    text = analysis.dumps_json({'a': float('nan'), 'b': [float('inf'), 1.5]})
    assert json.loads(text) == {'a': None, 'b': [None, 1.5]}


# ---------------------------------------------------------------------------
# Whole sweeps
# ---------------------------------------------------------------------------

def sensor_run():
    """Sensor sweep 20 and 35 % both directions with a glitch, a reversal, a gap and a stall."""
    rows, steps = [], []
    t = T0
    plan = [(20, 86.0), (-20, 80.0), (35, 150.0), (-35, 140.0)]
    for index, (percent, speed) in enumerate(plan):
        gap = (2.0, 0.23) if index == 2 else None
        step = make_step(index, percent, speed, t, 100, 1.0, 3.0, gap=gap)
        positions = [r.position_deg for r in step]
        if index == 0:
            # glitch at sample 40 lasting 3 samples (passes a median-of-5): two jumps
            for j in (40, 41, 42):
                positions[j] = (positions[j] + 40) % 360
        if index == 1:
            # two consecutive backward pairs (positive percent moves the angle down,
            # so negative percent moves it up; backward is down)
            i = 60
            base = positions[i - 1]
            positions[i] = base - 3
            positions[i + 1] = base - 6
            offset = base - 6 - positions[i + 1 + 1] + 2
            for j in range(i + 2, len(positions)):
                positions[j] = positions[j] + offset
        step = replace_positions(step, positions)
        rows.extend(step)
        steps.append({'index': index, 'kind': 'step', 'percent': percent,
                      'direction': 1 if percent > 0 else -1, 'end_reason': 'completed'})
        t = step[-1].t_host + 2.0
    # a stalled step at 20 %: no motion for 2.5 s, ended by the stall detector
    stall = [analysis.Sample(t + k * CYCLE, k, 200, 0.0, 0.0, 20, 1, 4, analysis.PHASE_SETTLE)
             for k in range(76)]
    rows.extend(stall)
    steps.append({'index': 4, 'kind': 'step', 'percent': 20, 'direction': 1,
                  'end_reason': 'stalled'})
    config = dict(CONFIG, min_percent_known={'positive_percent': 15, 'negative_percent': 15})
    return {'sweep': 'sensor', 'run_id': 'synthetic', 'status': 'complete',
            'config': config, 'steps': steps}, rows


def test_sensor_sweep_scores_jumps_reversals_stalls_and_histogram():
    run_info, rows = sensor_run()
    result = analysis.analyze_sensor_sweep(run_info, rows)
    assert result['angle_sign_for_positive_percent'] == -1
    assert result['angle_sign_consistent']
    steps = result['steps']
    assert [s['jump_count'] for s in steps] == [2, 0, 0, 0, 0]
    assert [s['reversal_count'] for s in steps] == [0, 1, 0, 0, 0]
    assert [s['stall_count'] for s in steps] == [0, 0, 0, 0, 1]
    assert steps[4]['stalled'] and not steps[0]['stalled']
    assert steps[2]['max_gap_s'] >= 0.23
    assert all(s['same_cycle_pairs'] > 0 for s in steps[:4])
    # out from the true angle (+40 against a -2.4 deg/sample motion), back from the
    # glitched one three samples later
    first, second = steps[0]['jumps']
    assert 35 <= first['delta_deg'] <= 41 and -45 <= second['delta_deg'] <= -39
    assert (second['start_deg'] - first['start_deg']) % 360 in range(30, 37)
    starts = [first['start_deg'], second['start_deg']]
    counts = result['jump_start_histogram']['counts']
    assert sum(counts) == 2
    for start in starts:
        assert counts[start // 10] >= 1
    # velocity sign follows the measured mapping and magnitude the synthetic speed
    assert steps[0]['velocity_rad_s'] == pytest.approx(-math.radians(86.0), rel=0.03)
    assert steps[2]['mean_velocity_rad_s'] == pytest.approx(-math.radians(150.0), rel=0.03)
    assert result['totals']['jumps'] == 2 and result['totals']['reversals'] == 1
    assert result['plugin_gate']['known'] is False   # no diagnostics snapshots in run.json
    assert result['suggested_pass'] is False
    json.loads(analysis.dumps_json(result))


def test_sensor_sweep_counts_plugin_gate_rejections_against_the_pass():
    run_info, rows = sensor_run()
    run_info['steps'] = run_info['steps'][1:3]          # the clean steps only
    rows = [r for r in rows if r.step_index in (1, 2)]
    # The sensor_run reversal is in step 1: make it a positive-percent step to drop it.
    run_info['steps'][0]['direction'] = 1
    base = {'sensor_diagnostics_before': {'reports_held': '0', 'reports_rejected': '0',
                                          'jumps_confirmed': '0', 'gate_resyncs': '0'}}
    clean = dict(run_info, **base, sensor_diagnostics_after={
        'reports_held': '1', 'reports_rejected': '0', 'jumps_confirmed': '1',
        'gate_resyncs': '0'})
    result = analysis.analyze_sensor_sweep(clean, rows)
    assert result['plugin_gate'] == {'reports_held': 1, 'reports_rejected': 0,
                                     'jumps_confirmed': 1, 'gate_resyncs': 0, 'known': True}
    assert result['suggested_pass'] == (result['totals']['jumps'] == 0 and
                                        result['totals']['reversals'] == 0)
    rejected = dict(run_info, **base, sensor_diagnostics_after={
        'reports_held': '3', 'reports_rejected': '2', 'jumps_confirmed': '0',
        'gate_resyncs': '0'})
    result = analysis.analyze_sensor_sweep(rejected, rows)
    assert result['plugin_gate']['reports_rejected'] == 2
    assert result['suggested_pass'] is False
    # An older plugin without the counters leaves the verdict to the telemetry.
    partial = dict(run_info, sensor_diagnostics_before={'sensor_errors': '0'},
                   sensor_diagnostics_after={'sensor_errors': '0'})
    assert analysis.analyze_sensor_sweep(partial, rows)['plugin_gate']['known'] is False


def test_sensor_sweep_gates_stalls_below_known_min_percent():
    run_info, rows = sensor_run()
    run_info['config']['min_percent_known'] = {'positive_percent': 25,
                                               'negative_percent': 25}
    result = analysis.analyze_sensor_sweep(run_info, rows)
    assert result['steps'][4]['stalled'] and result['steps'][4]['stall_count'] == 0
    assert result['steps'][1]['reversal_count'] == 0   # 20 % is below 25: not scored
    assert result['steps'][1]['scored_reversals_and_stalls'] is False


def speed_model(percent):
    """Positive percent: 0.030 (p - 12) from 18 %; negative: 0.027 (p - 13) from 21 %."""
    p = abs(percent)
    if percent > 0:
        return 0.030 * (p - 12) if p >= 18 else 0.0
    return 0.027 * (p - 13) if p >= 21 else 0.0


def speed_run(angle_sign=-1):
    rows, steps, t = [], [], T0
    entries = {1: [], -1: []}

    def run_step(kind, percent):
        nonlocal t
        index = len(steps)
        omega = speed_model(percent)
        direction = 1 if percent > 0 else -1
        if omega == 0.0:
            step = [analysis.Sample(t + k * CYCLE, k, 50, 0.0, 0.0, percent, direction, index,
                                    analysis.PHASE_SETTLE) for k in range(70)]
            reason = 'stalled'
        else:
            deg_s = math.degrees(omega)
            measure = analysis.measure_duration(deg_s, 3.0, 30.0)
            step = make_step(index, percent, deg_s, t, 50, 2.0, measure, angle_sign=angle_sign)
            reason = 'completed'
        rows.extend(step)
        steps.append({'index': index, 'kind': kind, 'percent': percent,
                      'direction': direction, 'end_reason': reason})
        entries[direction].append((abs(percent), reason == 'completed'))
        t = step[-1].t_host + 3.0

    for p in (10, 15, 20, 30, 50, 100):
        run_step('step', p)
        run_step('step', -p)
    for direction in (1, -1):
        while True:
            probe = analysis.next_search_probe(entries[direction])
            if probe is None:
                break
            run_step('search', probe * direction)
    run_info = {'sweep': 'speed', 'run_id': 'synthetic', 'status': 'complete',
                'config': dict(CONFIG), 'steps': steps}
    return run_info, rows


def test_speed_sweep_fit_min_percent_asymmetry_and_phase7():
    run_info, rows = speed_run()
    result = analysis.analyze_speed_sweep(run_info, rows)
    assert result['angle_sign_for_positive_percent'] == -1
    pos = result['directions']['positive_percent']
    neg = result['directions']['negative_percent']
    assert pos['min_percent'] == 18 and pos['min_percent_bracketed']
    assert neg['min_percent'] in (21, 22) and neg['min_percent_bracketed']
    assert pos['highest_stalled_percent'] >= 16
    assert pos['angle_sign'] == -1 and neg['angle_sign'] == 1
    assert pos['fit']['k_rad_s_per_percent'] == pytest.approx(0.030, rel=0.02)
    assert pos['fit']['p0_percent'] == pytest.approx(12.0, abs=1.0)
    assert pos['fit']['r_squared'] > 0.999
    assert neg['fit']['k_rad_s_per_percent'] == pytest.approx(0.027, rel=0.02)
    assert pos['omega_max_rad_s'] == pytest.approx(0.030 * 88, rel=0.02)
    assert pos['omega_max_measured_rad_s'] == pytest.approx(0.030 * 88, rel=0.02)
    assert neg['omega_max_rad_s'] == pytest.approx(0.027 * 87, rel=0.02)
    assert pos['velocity_at_min_percent_rad_s'] == pytest.approx(0.030 * 6, rel=0.05)
    assert result['asymmetry_ratio'] == pytest.approx((0.030 * 88) / (0.027 * 87), rel=0.03)
    assert result['recommended_max_speed_rad_per_sec'] == pytest.approx(neg['omega_max_rad_s'])
    # one revolution or more measured at speed
    fast = [s for s in result['steps'] if abs(s['percent']) == 100]
    assert all(s['revolutions'] >= 1.0 for s in fast)
    # Phase 7: a positive (angle-increasing) velocity needs a negative percent here.
    p7 = result['phase7']
    assert p7['positive_velocity']['percent_sign'] == -1
    assert p7['negative_velocity']['percent_sign'] == 1
    assert p7['positive_velocity']['k_percent_per_rad_s'] == pytest.approx(1 / 0.027, rel=0.02)
    assert p7['negative_velocity']['p0_percent'] == pytest.approx(12.0, abs=1.0)
    assert p7['negative_velocity']['min_percent'] == 18
    assert p7['max_speed_rad_per_sec'] == result['recommended_max_speed_rad_per_sec']
    assert result['totals']['jumps'] == 0
    assert result['totals']['stalled_steps'] >= 4


def test_speed_sweep_mirror_when_positive_percent_raises_the_angle():
    # Same dome with the opposite wiring/Inverted setting: everything mirrors.
    run_info, rows = speed_run(angle_sign=1)
    result = analysis.analyze_speed_sweep(run_info, rows)
    assert result['angle_sign_for_positive_percent'] == 1
    assert result['angle_sign_consistent']
    pos = result['directions']['positive_percent']
    neg = result['directions']['negative_percent']
    assert pos['angle_sign'] == 1 and neg['angle_sign'] == -1
    assert pos['wrong_direction_steps'] == [] and neg['wrong_direction_steps'] == []
    assert all(s['velocity_rad_s'] > 0 for s in result['steps']
               if s['moving'] and s['percent'] > 0)
    p7 = result['phase7']
    assert p7['positive_velocity']['percent_sign'] == 1
    assert p7['negative_velocity']['percent_sign'] == -1
    assert p7['positive_velocity']['k_percent_per_rad_s'] == pytest.approx(1 / 0.030, rel=0.02)
    assert p7['positive_velocity']['p0_percent'] == pytest.approx(12.0, abs=1.0)
    assert p7['positive_velocity']['min_percent'] == 18
    assert p7['negative_velocity']['k_percent_per_rad_s'] == pytest.approx(1 / 0.027, rel=0.02)
    assert p7['negative_velocity']['min_percent'] == neg['min_percent']
    # magnitudes are independent of the sign convention
    mirrored = analysis.analyze_speed_sweep(*speed_run(angle_sign=-1))
    assert result['asymmetry_ratio'] == pytest.approx(mirrored['asymmetry_ratio'])


def test_sensor_thresholds_from_speed_result():
    run_info, rows = speed_run()
    result = analysis.analyze_speed_sweep(run_info, rows)
    found = analysis.sensor_thresholds_from_speed_result(result, True, 100)
    assert found is not None
    assert found[0] == pytest.approx(1.5 * math.degrees(result['max_measured_speed_rad_s']))
    assert found[1] == {'positive_percent': 18,
                        'negative_percent': result['directions']['negative_percent'][
                            'min_percent']}
    assert analysis.sensor_thresholds_from_speed_result(result, False, 100) is None
    result['highest_moving_percent'] = 50
    assert analysis.sensor_thresholds_from_speed_result(result, True, 100) is None
    result['status'] = 'aborted'
    assert analysis.sensor_thresholds_from_speed_result(result, True, 50) is None


def test_partial_run_and_missing_metadata():
    run_info, rows = speed_run()
    run_info['status'] = 'aborted'
    run_info['steps'] = run_info['steps'][:3]   # later steps only in the CSV
    cut = [r for r in rows if r.step_index <= 4]
    result = analysis.analyze_speed_sweep(run_info, cut)
    assert result['partial'] is True
    assert [s['index'] for s in result['steps']] == [0, 1, 2, 3, 4]
    assert result['steps'][4]['percent'] == speed_run()[0]['steps'][4]['percent']


def write_run(tmp_path, run_info, rows):
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    analysis.write_json(str(run_dir / analysis.RUN_JSON), run_info)
    analysis.write_csv(str(run_dir / analysis.SAMPLES_CSV), rows)
    return run_dir


# ---------------------------------------------------------------------------
# Position sweep
# ---------------------------------------------------------------------------

def _rows(index, t0, percent, direction, phase, profile, seq0=0):
    """Rows from a list of (t_rel, unwrapped_deg) points, quantised like the plugin."""
    rows = []
    for k, (t_rel, angle) in enumerate(profile):
        rows.append(analysis.Sample(
            t_host=t0 + host_stamp(t_rel), seq=seq0 + k, position_deg=int(round(angle)) % 360,
            unwrapped_rad=math.radians(angle), velocity_rad_s=float('nan'),
            commanded_percent=percent, direction=direction, step_index=index, phase=phase))
    return rows


def stop_profile(speed, decel, run_s=1.0):
    """Constant speed for run_s, then a constant deceleration to rest."""
    points = []
    t = 0.0
    while t < run_s:
        points.append((t, 'run', speed * t))
        t += REPORT_DT
    t_stop = t
    while True:
        tau = t - t_stop
        v = max(0.0, speed - decel * tau)
        u = speed * t_stop + speed * tau - 0.5 * decel * tau * tau
        if v <= 0.0:
            u = speed * t_stop + speed * speed / (2.0 * decel)
        points.append((t, 'coast', u))
        if v <= 0.0 and tau > speed / decel + 1.5:
            break
        t += REPORT_DT
    return points


def position_run():
    rows, steps = [], []
    t = T0
    # Two stop steps: 200 deg/s (+50 %) and 100 deg/s (-35 %), both stopping at 400 deg/s^2:
    # stop distances 50 and 12.5 deg.
    for index, (percent, speed) in enumerate([(50, 200.0), (-35, 100.0)]):
        sign = -1 if percent > 0 else 1   # positive percent moves the angle down
        profile = stop_profile(speed, 400.0)
        run = [(tr, 100 + sign * u) for tr, ph, u in profile if ph == 'run']
        coast = [(tr, 100 + sign * u) for tr, ph, u in profile if ph == 'coast']
        direction = 1 if percent > 0 else -1
        rows += _rows(index, t, percent, direction, analysis.PHASE_RUN, run)
        rows += _rows(index, t, 0, direction, analysis.PHASE_COAST, coast, seq0=len(run))
        steps.append({'index': index, 'kind': 'stop', 'percent': percent,
                      'direction': direction, 'end_reason': 'completed', 'settled': True,
                      't_stop_wall': t + run[-1][0] + 0.01})
        t = rows[-1].t_host + 2.0
    # Two moves: +90 at 120 deg/s overshooting by 3 deg, -40 stopping 2 deg short.
    for index, (delta, speed, past) in enumerate([(90, 120.0, 3), (-40, 60.0, -2)], start=2):
        start = 10 if delta > 0 else 103
        target = (start + delta) % 360
        direction = 1 if delta > 0 else -1
        end = start + delta + past * direction
        profile = []
        tr = 0.0
        while abs(speed * tr) < abs(end - start):
            profile.append((tr, start + direction * speed * tr))
            tr += REPORT_DT
        while tr < abs(end - start) / speed + 1.6:
            profile.append((tr, end))
            tr += REPORT_DT
        rows += _rows(index, t, 0, direction, analysis.PHASE_MOVE, profile)
        steps.append({'index': index, 'kind': 'move', 'percent': 0, 'direction': direction,
                      'start_deg': start, 'target_deg': target, 'delta_deg': delta,
                      'speed_percent': 45, 'end_reason': 'completed', 'settled': True,
                      't_move_wall': t - 0.02,
                      'firmware': {'status_seen': True, 'position_timeouts': 0,
                                   'reached_target_deg': target}})
        t = rows[-1].t_host + 2.0
    config = dict(CONFIG, raw_motor=False, stopped_deg_s=5.0, wrap_allowance_deg=10.0,
                  fudge_deg=5, move_speed_percent=45)
    run_info = {'sweep': 'position', 'run_id': 'synthetic', 'status': 'complete',
                'config': config, 'steps': steps,
                'settings_context': {'speed_target': 60, 'deceleration_scale': 250, 'fudge': 5}}
    return run_info, rows


def test_position_sweep_stop_curve_and_moves():
    run_info, rows = position_run()
    result = analysis.analyze_position_sweep(run_info, rows)
    assert result['settings']['fudge'] == 5
    stops = result['stop_curve']['steps']
    assert [s['kind'] for s in stops] == ['stop', 'stop']
    assert all(s['stop_found'] for s in stops)
    assert stops[0]['speed_before_stop_deg_s'] == pytest.approx(200.0, abs=8.0)
    assert stops[0]['stop_distance_deg'] == pytest.approx(50.0, abs=4.0)
    assert stops[1]['speed_before_stop_deg_s'] == pytest.approx(100.0, abs=6.0)
    assert stops[1]['stop_distance_deg'] == pytest.approx(12.5, abs=3.0)
    assert 0.2 < stops[0]['stop_time_s'] < 0.75
    fit = result['stop_curve']['fit']
    assert fit['points'] == 2
    # d = v^2 / 800 through (100, 12.5) and (200, 50): slope 0.375 s, intercept -25 deg
    assert fit['slope'] == pytest.approx(0.375, abs=0.05)
    assert result['stop_curve']['fit_positive_percent'] is None   # one point per direction
    assert result['stop_curve']['fit_steps'] == [0, 1]
    assert not stops[0]['pre_stop_crosses_zero'] and not stops[0]['coast_crosses_zero']

    moves = result['moves']
    assert moves['count'] == 2 and moves['completed'] == 2 and moves['timed_out'] == 0
    plus, minus = moves['steps']
    assert result['move_speed_percent'] == 45
    assert plus['speed_percent'] == 45 and moves['steps'][1]['speed_percent'] == 45
    assert plus['target_deg'] == 100 and plus['rest_error_deg'] == 3
    assert not plus['target_in_dead_zone']
    assert plus['furthest_past_target_deg'] == 3 and plus['reached_target']
    assert plus['peak_deg_s'] == pytest.approx(120.0, abs=10.0)
    assert 0.6 < plus['settle_s'] < 1.1
    assert plus['arrival_deg_s'] == pytest.approx(120.0, abs=15.0)
    assert minus['target_deg'] == 63 and minus['rest_error_deg'] == 2
    assert minus['furthest_past_target_deg'] == -2 and not minus['reached_target']
    assert moves['never_reached_target'] == 1
    assert moves['rest_error_deg'] == {'count': 2, 'max_abs': 3, 'mean_abs': 2.5}
    assert moves['rest_error_deg_angle_increasing']['max_abs'] == 3
    assert moves['rest_error_deg_angle_decreasing']['max_abs'] == 2
    assert moves['overshoot_max_deg'] == 3
    assert moves['firmware_timeouts'] == 0
    assert result['totals']['jumps'] == 0
    assert analysis.analyze_run(run_info, rows) == result


def test_position_sweep_tolerates_missing_stop_and_empty_move():
    run_info, rows = position_run()
    # A stop step whose percent never showed in telemetry and a move with no samples.
    rows = [r._replace(commanded_percent=0) if r.step_index == 0 else r for r in rows]
    rows = [r for r in rows if r.step_index != 3]
    run_info['steps'][0]['end_reason'] = 'stalled'
    result = analysis.analyze_position_sweep(run_info, rows)
    first = result['stop_curve']['steps'][0]
    assert first['stop_found'] is True          # falls back to the first coast sample
    assert result['stop_curve']['fit_steps'] == [1]   # a stalled step is not fitted
    assert result['stop_curve']['fit'] is None
    empty = result['moves']['steps'][1]
    assert empty['samples'] == 0 and not empty['reached_target']
    assert result['moves']['completed'] == 2    # end_reason from run.json
    json.loads(analysis.dumps_json(result))     # NaNs are nulls


@pytest.mark.parametrize('make_run', [sensor_run, speed_run, position_run])
def test_run_dir_reproduces_and_report_cli_checks(tmp_path, make_run):
    run_info, rows = make_run()
    run_dir = write_run(tmp_path, run_info, rows)
    result = analysis.analyze_run_dir(str(run_dir))
    assert analysis.dumps_json(result) == analysis.dumps_json(
        analysis.analyze_run(run_info, rows))
    saved = run_dir / analysis.result_filename(run_info['sweep'])
    analysis.write_json(str(saved), result)

    env = dict(os.environ, PYTHONPATH=PKG_ROOT + os.pathsep + os.environ.get('PYTHONPATH', ''))
    script = os.path.join(PKG_ROOT, 'scripts', 'roamadome_bench_report')
    proc = subprocess.run([sys.executable, script, str(run_dir), '--check'], env=env,
                          capture_output=True, text=True, check=False)
    assert proc.returncode == 0, proc.stderr
    assert json.loads(proc.stdout) == json.loads(saved.read_text())
    proc = subprocess.run([sys.executable, script, str(run_dir), '--check',
                           '--set', 'max_plausible_deg_s=1'], env=env,
                          capture_output=True, text=True, check=False)
    assert proc.returncode == 1


def test_analysis_imports_without_rclpy():
    code = ('import sys; import ros2_roamadome.bench.analysis; '
            'sys.exit(1 if "rclpy" in sys.modules else 0)')
    env = dict(os.environ, PYTHONPATH=PKG_ROOT)
    proc = subprocess.run([sys.executable, '-B', '-c', code], env=env, check=False)
    assert proc.returncode == 0
