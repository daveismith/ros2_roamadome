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

"""Unit tests for ros2_roamadome.bench.control (node safety logic, no ROS graph)."""

import os
import subprocess
import sys
import threading
import time

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from ros2_roamadome.bench import control  # noqa: E402


class FakeDrive:
    """Records bench/drive calls; optionally blocks inside a call."""

    def __init__(self, ok=True, message='ok'):
        self.calls = []
        self.ok = ok
        self.message = message
        self.block = None      # threading.Event the call waits on
        self.entered = threading.Event()

    def __call__(self, percent, timeout):
        self.calls.append(percent)
        self.entered.set()
        if self.block is not None:
            self.block.wait(5.0)
        return self.ok, self.message


# ---------------------------------------------------------------------------
# SweepClaim
# ---------------------------------------------------------------------------

def test_sweep_claim_is_exclusive_and_releasable():
    claim = control.SweepClaim()
    assert claim.try_claim()
    assert claim.held
    assert not claim.try_claim()
    claim.release()
    assert not claim.held
    assert claim.try_claim()


def test_sweep_claim_only_one_of_many_concurrent_starts_wins():
    claim = control.SweepClaim()
    barrier = threading.Barrier(16)
    wins = []

    def start():
        barrier.wait()
        if claim.try_claim():
            wins.append(1)

    threads = [threading.Thread(target=start) for _ in range(16)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len(wins) == 1


# ---------------------------------------------------------------------------
# DriveGate
# ---------------------------------------------------------------------------

def test_gate_refuses_non_zero_once_stopping_but_sends_zero():
    drive = FakeDrive()
    gate = control.DriveGate(drive)
    gate.arm()
    assert gate.send(35) == (True, 'ok')
    assert gate.percent == 35
    gate.begin_stopping()
    ok, message = gate.send(35)
    assert not ok and message == control.REFUSED_STOPPING
    assert drive.calls == [35]          # the refused call never reached the service
    assert gate.send(0) == (True, 'ok')
    assert drive.calls == [35, 0]
    gate.arm()                          # next sweep
    assert gate.send(20)[0]


def test_gate_heartbeat_resends_percent_and_sends_zero_once_when_stopping():
    drive = FakeDrive()
    gate = control.DriveGate(drive)
    gate.arm()
    assert gate.heartbeat() is None     # nothing commanded
    gate.send(35)
    assert gate.heartbeat() == (True, 'ok')
    assert drive.calls == [35, 35]
    gate.begin_stopping()
    assert gate.heartbeat() == (True, 'ok')
    assert drive.calls[-1] == 0
    assert gate.heartbeat() is None     # 0 is not repeated
    assert drive.calls == [35, 35, 0]


def test_gate_failed_send_clears_percent_so_heartbeat_stays_quiet():
    drive = FakeDrive(ok=False, message='a controller is commanding the dome')
    gate = control.DriveGate(drive)
    gate.arm()
    assert gate.send(35) == (False, 'a controller is commanding the dome')
    assert gate.percent == 0
    assert gate.heartbeat() is None


def test_gate_stop_requested_during_an_in_flight_send_is_honoured():
    drive = FakeDrive()
    drive.block = threading.Event()
    gate = control.DriveGate(drive)
    gate.arm()
    worker = threading.Thread(target=gate.send, args=(35,))
    worker.start()
    assert drive.entered.wait(2.0)
    assert gate.heartbeat() is None      # the worker holds the lock
    gate.begin_stopping()                # abort arrives mid-call
    drive.block.set()
    worker.join(2.0)
    # The non-zero percent reached the service; the next heartbeat sends 0.
    assert gate.heartbeat() == (True, 'ok')
    assert drive.calls == [35, 0]


def test_gate_force_stop_takes_the_lock_when_free_and_sends_zero_anyway_when_held():
    drive = FakeDrive()
    gate = control.DriveGate(drive)
    gate.arm()
    gate.send(35)
    assert gate.force_stop(lock_timeout=0.1, timeout=0.1) == (True, 'ok')
    assert drive.calls == [35, 0] and gate.stopping

    drive = FakeDrive()
    drive.block = threading.Event()
    gate = control.DriveGate(drive)
    gate.arm()
    worker = threading.Thread(target=gate.send, args=(35,))
    worker.start()
    assert drive.entered.wait(2.0)
    start = time.monotonic()
    drive.block.set()                    # let the stuck call and ours return
    gate.force_stop(lock_timeout=0.2, timeout=0.1)
    worker.join(2.0)
    assert time.monotonic() - start < 2.0
    assert drive.calls[-1] == 0


# ---------------------------------------------------------------------------
# DriveGate: bench moves
# ---------------------------------------------------------------------------

class FakeMove:
    def __init__(self, ok=True, message='move ok'):
        self.calls = []
        self.ok = ok
        self.message = message

    def __call__(self, target_deg, timeout):
        self.calls.append(target_deg)
        return self.ok, self.message


def test_gate_move_is_kept_alive_by_the_heartbeat_and_released_by_zero():
    drive, move = FakeDrive(), FakeMove()
    gate = control.DriveGate(drive, move)
    gate.arm()
    assert gate.move(120) == (True, 'move ok')
    assert gate.move_target == 120
    assert gate.heartbeat() == (True, 'move ok')      # keep-alive repeats the target
    assert move.calls == [120, 120]
    assert gate.move(480) == (True, 'move ok')        # wrapped
    assert move.calls[-1] == 120 and gate.move_target == 120
    ok, message = gate.send(20)
    assert not ok and message == control.REFUSED_MOVE_ACTIVE
    assert drive.calls == []
    assert gate.send(0) == (True, 'ok')               # releases the claim
    assert gate.move_target is None
    assert drive.calls == [0]
    assert gate.heartbeat() is None


def test_gate_move_refused_while_driving_stopping_or_unsupported():
    drive, move = FakeDrive(), FakeMove()
    gate = control.DriveGate(drive, move)
    gate.arm()
    gate.send(35)
    assert gate.move(10) == (False, control.REFUSED_DRIVE_ACTIVE)
    gate.send(0)
    assert gate.move(10)[0]
    gate.begin_stopping()
    assert gate.move(20) == (False, control.REFUSED_STOPPING)
    assert gate.heartbeat() == (True, 'ok')           # stopping: one 0 releases the move
    assert drive.calls[-1] == 0 and gate.move_target is None
    assert gate.heartbeat() is None
    assert control.DriveGate(FakeDrive()).move(10) == (False, 'bench move is not supported')


def test_gate_failed_move_is_not_kept_alive():
    drive, move = FakeDrive(), FakeMove(ok=False, message='a controller is commanding the dome')
    gate = control.DriveGate(drive, move)
    gate.arm()
    assert gate.move(10) == (False, 'a controller is commanding the dome')
    assert gate.move_target is None
    assert gate.heartbeat() is None


def test_move_outcome_from_motion_status():
    before = {'position_timeouts': '2', 'last_target_reached_deg': '100',
              'last_move_completed_age_s': '40.0'}
    after = {'position_timeouts': '2', 'last_target_reached_deg': '250',
             'last_move_completed_age_s': '1.2'}
    assert control.move_outcome(before, after) == {
        'status_seen': True, 'position_timeouts': 0, 'reached_target_deg': 250}
    timed_out = {'position_timeouts': '3', 'last_target_reached_deg': '100',
                 'last_move_completed_age_s': '52.0'}
    assert control.move_outcome(before, timed_out) == {
        'status_seen': True, 'position_timeouts': 1, 'reached_target_deg': None}
    # A move to the same target as the last one still counts if completed recently.
    same = dict(after, last_target_reached_deg='100')
    assert control.move_outcome(before, same)['reached_target_deg'] == 100
    assert control.move_outcome(before, before) == {
        'status_seen': False, 'position_timeouts': None, 'reached_target_deg': None}
    assert control.move_outcome({}, {'last_target_reached_deg': 'unknown'}) == {
        'status_seen': True, 'position_timeouts': None, 'reached_target_deg': None}


# ---------------------------------------------------------------------------
# Parameter validation
# ---------------------------------------------------------------------------

DEFAULTS = {
    'heartbeat_period_s': 0.5,
    'velocity_window_s': 0.5,
    'sensor_sweep.percents': [20, 35, 50, 75, 100],
    'sensor_sweep.duration_s': 30.0,
    'sensor_sweep.settle_s': 2.0,
    'sensor_sweep.max_plausible_deg_s': 720.0,
    'speed_sweep.percents': [10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100],
    'speed_sweep.settle_s': 2.0,
    'speed_sweep.min_measure_s': 4.0,
    'speed_sweep.max_measure_s': 30.0,
    'speed_sweep.stall_deg_s': 3.0,
    'speed_sweep.stall_s': 2.0,
    'sensor_sweep.wrap_allowance_deg': 10.0,
    'position_sweep.stop_percents': [35, 50, 70, 100],
    'position_sweep.stop_run_s': 5.0,
    'position_sweep.moves_deg': [150, -150, 40, -40],
    'position_sweep.settle_s': 1.2,
    'position_sweep.move_timeout_s': 12.0,
    'position_sweep.stopped_deg_s': 5.0,
}


def errors_with(**overrides):
    values = dict(DEFAULTS)
    values.update({k.replace('__', '.'): v for k, v in overrides.items()})
    return control.validate_config(values.__getitem__)


def test_defaults_are_valid():
    assert errors_with() == []


def test_heartbeat_period_must_be_within_the_watchdog():
    assert errors_with(heartbeat_period_s=1.0) == []
    assert errors_with(heartbeat_period_s=1.2)
    assert errors_with(heartbeat_period_s=0.0)
    assert errors_with(heartbeat_period_s=-0.5)


def test_bad_percent_lists_are_rejected_not_fixed():
    assert errors_with(sensor_sweep__percents=[20, 35]) == []
    assert errors_with(sensor_sweep__percents=[])
    assert errors_with(sensor_sweep__percents=[0, 20])
    assert errors_with(sensor_sweep__percents=[20, 101])
    assert errors_with(sensor_sweep__percents=[-20, 35])
    assert errors_with(speed_sweep__percents=[20, 20, 30])
    assert errors_with(speed_sweep__percents=[30, 20])
    assert any('ascending' in e for e in errors_with(speed_sweep__percents=[30, 20]))
    assert any('duplicates' in e for e in errors_with(speed_sweep__percents=[20, 20]))


def test_position_sweep_parameters_are_validated():
    assert errors_with(position_sweep__moves_deg=[-359, 359, 1]) == []
    assert errors_with(position_sweep__moves_deg=[])
    assert errors_with(position_sweep__moves_deg=[150, 0])
    assert errors_with(position_sweep__moves_deg=[360])
    assert errors_with(position_sweep__moves_deg=[True])
    assert errors_with(position_sweep__stop_percents=[50, 35])
    assert errors_with(position_sweep__stop_run_s=0.0)
    assert errors_with(position_sweep__settle_s=0.0)
    assert errors_with(position_sweep__move_timeout_s=-1.0)
    assert errors_with(position_sweep__stopped_deg_s=0.0)
    assert errors_with(sensor_sweep__wrap_allowance_deg=0.0) == []
    assert errors_with(sensor_sweep__wrap_allowance_deg=-1.0)


def test_bad_durations_are_rejected():
    assert errors_with(sensor_sweep__settle_s=0.0) == []
    assert errors_with(sensor_sweep__settle_s=-1.0)
    assert errors_with(speed_sweep__settle_s=-0.1)
    assert errors_with(sensor_sweep__duration_s=0.0)
    assert errors_with(speed_sweep__stall_s=-2.0)
    assert errors_with(speed_sweep__min_measure_s=31.0)
    assert any('must not exceed' in e for e in errors_with(speed_sweep__min_measure_s=31.0))


# ---------------------------------------------------------------------------
# Settings ack confirmation
# ---------------------------------------------------------------------------

def test_parse_settings_counters():
    values = {'settings_writes_pending': '1', 'settings_acks': '4',
              'settings_ack_timeouts': '0', 'command_mode': 'idle'}
    assert control.parse_settings_counters(values) == (1, 4, 0)
    assert control.parse_settings_counters({'command_mode': 'idle'}) is None
    assert control.parse_settings_counters(dict(values, settings_acks='x')) is None


def test_settings_ack_state():
    base = (0, 10, 2)
    assert control.settings_ack_state(base, (3, 10, 2), 3) == 'waiting'
    assert control.settings_ack_state(base, (1, 12, 2), 3) == 'waiting'
    assert control.settings_ack_state(base, (0, 12, 2), 3) == 'waiting'   # one ack missing
    assert control.settings_ack_state(base, (0, 13, 2), 3) == 'acked'
    assert control.settings_ack_state(base, (0, 11, 3), 3) == 'timeout'
    assert control.settings_ack_state(base, (0, 10, 2), 0) == 'acked'
    assert control.settings_ack_state(base, (1, 10, 2), 0) == 'waiting'  # someone else's


# ---------------------------------------------------------------------------
# Stop failure advice
# ---------------------------------------------------------------------------

def test_stop_failure_advice_distinguishes_already_stopped_from_heartbeat():
    inactive = control.stop_failure_advice(
        'hardware interface is inactive; bench drive rejected')
    controller = control.stop_failure_advice(
        'a controller is commanding the dome (mode is not IDLE); bench drive rejected')
    missing = control.stop_failure_advice('/roamadome/bench/drive is not available')
    assert 'already stopped by the plugin' in inactive
    assert 'already stopped by the plugin' in controller
    assert 'heartbeat' not in inactive
    assert 'heartbeat' in missing and 'already stopped' not in missing
    assert 'serial port closed' in control.stop_failure_advice(
        'serial port is closed; bench drive rejected')


def test_control_imports_without_rclpy():
    code = ('import sys; import ros2_roamadome.bench.control; '
            'sys.exit(1 if "rclpy" in sys.modules else 0)')
    env = dict(os.environ, PYTHONPATH=PKG_ROOT)
    assert subprocess.run([sys.executable, '-B', '-c', code], env=env,
                          check=False).returncode == 0
