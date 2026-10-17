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

"""
The ``roamadome_bench`` node: sensor, speed and position sweeps for the dome.

See docs/dome_diagnostics_plan.md sections 5.4, 6 (phase 5) and 7, and the
README's "Position-move tuning" for the position sweep. The node drives the
dome only through the hardware node's ``bench/drive`` and ``bench/move``
services, reads ``telemetry`` and changes ``device.*`` parameters through the
hardware node's parameter services. The hardware node is found by name through the
``hardware_node`` parameter (default ``roamadome``).

Threads: the executor (several threads) runs the service callbacks, the
telemetry subscription, the status timer and the heartbeat timer; a single
worker thread per sweep runs the steps, the stop and the settings restore.
Service callbacks only start, abort and report.
"""

import glob
import json
import math
import os
import queue
import signal
import sys
import threading
import time
import traceback

from astromech_msgs.msg import DomeTelemetry
from astromech_msgs.srv import DomeBenchDrive
from astromech_msgs.srv import DomeBenchMove
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import analysis
from . import control

# Settings the bench may change, saved to settings_before.json (plan 6, phase 5 step 1).
SAVED_SETTINGS = ('max_speed', 'min_speed', 'speed_target', 'scaling')
# Recorded for context only; never written (Inverted, HomePos, AutoSafety are off limits).
CONTEXT_SETTINGS = ('speed_home', 'acceleration_scale', 'deceleration_scale', 'fudge',
                    'timeout', 'inverted', 'home_pos', 'auto_safety')
# raw_motor: max speed 100, target speed 100, scaling off (plan 5.4).
RAW_MOTOR_SETTINGS = (('max_speed', 100), ('speed_target', 100), ('scaling', False))

TELEMETRY_TIMEOUT_S = 1.0      # dropout during a sweep aborts it
STANDSTILL_TIMEOUT_S = 5.0     # plan: wait for standstill or 5 s
STOP_SETTLE_TIMEOUT_S = 8.0    # position sweep: coasting to rest after a stop
MOTION_STATUS_WAIT_S = 2.5     # wait for the 1 Hz motion status after a move
SERVICE_TIMEOUT_S = control.SERVICE_TIMEOUT_S  # any single service call
SETTING_READBACK_TIMEOUT_S = 5.0
SETTING_RETRY_S = 10.0         # retry a write the plugin refuses while the dome is stopping
SETTINGS_IN_FLIGHT_RETRY_S = 12.0  # first drive after a settings write waits for its ack
SETTINGS_SETTLE_S = 2.0        # let the post-ack #DPCONFIG refresh land before reading back
SETTINGS_ACK_TIMEOUT_S = 10.0  # wait for the firmware to ack the settings writes
DIAGNOSTICS_WAIT_S = 3.0       # wait for a first roamadome: motion status (1 Hz)
LOOP_PERIOD_S = 0.05
SPEED_ESTIMATE_S = 1.0         # tail of the settle phase used to size the measure window

RETRYABLE_SET_REASONS = ('rotating', 'bench drive is active')


class SweepAbort(Exception):
    """Raised inside the worker to end a sweep early (abort, dropout, rejection)."""


def _now_iso():
    return time.strftime('%Y-%m-%dT%H:%M:%S%z')


def _same_value(a, b):
    return a == b and isinstance(a, bool) == isinstance(b, bool)


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class BenchNode(Node):
    """Bench diagnostics node; see the module docstring."""

    def __init__(self):
        super().__init__('roamadome_bench')
        self.declare_parameter('hardware_node', 'roamadome')
        self.declare_parameter('output_dir', '~/dome_bench')
        self.declare_parameter('heartbeat_period_s', 0.5)
        self.declare_parameter('velocity_window_s', 0.5)
        self.declare_parameter('sensor_sweep.percents', [20, 35, 50, 75, 100])
        self.declare_parameter('sensor_sweep.duration_s', 30.0)
        self.declare_parameter('sensor_sweep.settle_s', 2.0)
        self.declare_parameter('sensor_sweep.max_plausible_deg_s', 720.0)
        self.declare_parameter('sensor_sweep.wrap_allowance_deg', 10.0)
        self.declare_parameter('speed_sweep.percents',
                               [10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100])
        self.declare_parameter('speed_sweep.settle_s', 2.0)
        self.declare_parameter('speed_sweep.min_measure_s', 4.0)
        self.declare_parameter('speed_sweep.max_measure_s', 30.0)
        self.declare_parameter('speed_sweep.stall_deg_s', 3.0)
        self.declare_parameter('speed_sweep.stall_s', 2.0)
        self.declare_parameter('speed_sweep.raw_motor', True)
        self.declare_parameter('position_sweep.stop_percents', [35, 50, 70, 100])
        self.declare_parameter('position_sweep.stop_run_s', 5.0)
        self.declare_parameter('position_sweep.moves_deg',
                               [150, -150, 40, -40, 90, -90, 20, -20, 120, -120, 60, -60])
        self.declare_parameter('position_sweep.settle_s', 1.2)
        self.declare_parameter('position_sweep.move_timeout_s', 12.0)
        self.declare_parameter('position_sweep.stopped_deg_s', 5.0)

        errors = control.validate_config(lambda name: self.get_parameter(name).value)
        if errors:
            raise ValueError('invalid roamadome_bench parameters:\n  ' + '\n  '.join(errors))
        self.add_on_set_parameters_callback(self._on_set_parameters)

        hw = str(self.get_parameter('hardware_node').value).strip().rstrip('/')
        self._hw = hw if hw.startswith('/') else '/' + hw

        self._lock = threading.Lock()          # status, latest telemetry and diagnostics
        self._claim = control.SweepClaim()     # one sweep at a time, claimed atomically
        # Serialises bench/drive and bench/move calls; one at a time.
        self._gate = control.DriveGate(self._call_drive, self._call_move)
        self._abort_event = threading.Event()
        self._force_event = threading.Event()  # second signal: stop waiting for cleanup
        self._abort_reason = None
        self._sweep_active = False
        self._telemetry_queue = queue.Queue()
        self._latest_msg = None
        self._latest_rx = 0.0
        self._sensor_diag = None
        self._motion_counters = None           # (pending, acks, timeouts) from /diagnostics
        self._motion_values = {}               # every value of the latest roamadome: motion
        self._motion_stamp = 0.0               # header stamp of that diagnostics message
        self._status = {'state': 'idle'}
        self._last_result = None

        self._group = ReentrantCallbackGroup()
        self._heartbeat_group = MutuallyExclusiveCallbackGroup()

        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(DomeTelemetry, self._hw + '/telemetry', self._on_telemetry,
                                 qos, callback_group=self._group)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10,
                                 callback_group=self._group)
        self._drive_client = self.create_client(
            DomeBenchDrive, self._hw + '/bench/drive', callback_group=self._group)
        self._move_client = self.create_client(
            DomeBenchMove, self._hw + '/bench/move', callback_group=self._group)
        self._get_client = self.create_client(
            GetParameters, self._hw + '/get_parameters', callback_group=self._group)
        self._set_client = self.create_client(
            SetParameters, self._hw + '/set_parameters', callback_group=self._group)
        self.create_service(Trigger, '~/start_sensor_sweep',
                            lambda req, res: self._on_start('sensor', res),
                            callback_group=self._group)
        self.create_service(Trigger, '~/start_speed_sweep',
                            lambda req, res: self._on_start('speed', res),
                            callback_group=self._group)
        self.create_service(Trigger, '~/start_position_sweep',
                            lambda req, res: self._on_start('position', res),
                            callback_group=self._group)
        self.create_service(Trigger, '~/abort', self._on_abort, callback_group=self._group)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        self.create_timer(0.5, self._publish_status, callback_group=self._group)
        heartbeat = float(self.get_parameter('heartbeat_period_s').value)
        if not heartbeat > 0.0:
            heartbeat = 0.5
        self._heartbeat_period = heartbeat
        self.create_timer(heartbeat, self._on_heartbeat, callback_group=self._heartbeat_group)
        self.get_logger().info(
            'roamadome_bench ready: hardware node %s (telemetry %s/telemetry, '
            'drive %s/bench/drive), heartbeat %.2f s' % (self._hw, self._hw, self._hw, heartbeat))

    # ------------------------------------------------------------------
    # Executor callbacks
    # ------------------------------------------------------------------

    def _on_telemetry(self, msg):
        rx = time.monotonic()
        with self._lock:
            self._latest_msg = msg
            self._latest_rx = rx
            active = self._sweep_active
        if active:
            self._telemetry_queue.put(msg)

    def _on_set_parameters(self, parameters):
        result = SetParametersResult(successful=True)
        proposed = {p.name: p.value for p in parameters}
        for name in ('hardware_node', 'heartbeat_period_s'):
            if name in proposed:
                result.successful = False
                result.reason = '%s is read at startup; restart the node to change it' % name
                return result
        errors = control.validate_config(
            lambda name: proposed[name] if name in proposed else self.get_parameter(name).value)
        if errors:
            result.successful = False
            result.reason = '; '.join(errors)
        return result

    def _on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'roamadome: motion':
                values = {kv.key: kv.value for kv in status.values}
                counters = control.parse_settings_counters(values)
                with self._lock:
                    self._motion_values = values
                    if counters is not None:
                        self._motion_counters = counters
                        self._motion_stamp = _stamp_seconds(msg.header.stamp)
            if status.name == 'roamadome: sensor':
                values = {kv.key: kv.value for kv in status.values}
                values['level'] = int.from_bytes(status.level, 'little') \
                    if isinstance(status.level, bytes) else int(status.level)
                values['message'] = status.message
                with self._lock:
                    self._sensor_diag = values

    def _on_start(self, sweep, response):
        response.success = False
        # Claim the sweep slot atomically before any other check: the start
        # services run in a reentrant group, so two calls can overlap.
        if not self._claim.try_claim():
            response.message = 'a sweep is already running'
            return response
        # Reset the abort state right after the claim, so an abort that arrives while
        # the start is being prepared is kept and ends the sweep at its first check.
        self._abort_event.clear()
        self._abort_reason = None
        self._gate.arm()
        try:
            params, run_dir = self._prepare_start(sweep)
        except (ValueError, OSError) as exc:
            self._claim.release()
            response.message = str(exc)
            return response

        while not self._telemetry_queue.empty():
            self._telemetry_queue.get_nowait()
        worker = threading.Thread(target=self._sweep_main, args=(sweep, params, run_dir),
                                  name='roamadome_bench_%s' % sweep, daemon=True)
        with self._lock:
            self._sweep_active = True
            self._status = {'state': 'running', 'sweep': sweep, 'run_dir': run_dir,
                            'phase': 'starting', 'started_mono': time.monotonic()}
        try:
            worker.start()
        except RuntimeError as exc:
            with self._lock:
                self._sweep_active = False
                self._status = {'state': 'idle'}
            self._claim.release()
            response.message = 'could not start the sweep thread: %s' % exc
            return response
        response.success = True
        response.message = '%s sweep started; results in %s' % (sweep, run_dir)
        self.get_logger().info(response.message)
        return response

    def _prepare_start(self, sweep):
        """Pre-flight checks, parameters and run directory; raises ValueError to reject."""
        with self._lock:
            latest = self._latest_msg
            age = time.monotonic() - self._latest_rx
        if latest is None or age > TELEMETRY_TIMEOUT_S:
            raise ValueError('no telemetry on %s/telemetry (is publish_telemetry on?)' % self._hw)
        if latest.command_mode != DomeTelemetry.COMMAND_MODE_IDLE or latest.commanded_percent:
            raise ValueError('hardware is not idle (command_mode %d, commanded %d%%); a '
                             'controller or another bench client holds the dome'
                             % (latest.command_mode, latest.commanded_percent))
        clients = [(self._drive_client, 'bench/drive'), (self._get_client, 'get_parameters'),
                   (self._set_client, 'set_parameters')]
        if sweep == 'position':
            clients.append((self._move_client, 'bench/move'))
        for client, name in clients:
            if not client.service_is_ready():
                raise ValueError('%s/%s is not available (bench mode off?)' % (self._hw, name))
        params = self._read_sweep_params(sweep)
        return params, self._make_run_dir(params['output_dir'], sweep)

    def _on_abort(self, request, response):
        if not self._claim.held:
            response.success = False
            response.message = 'no sweep is running'
            return response
        self.request_abort('abort service')
        response.success = True
        response.message = 'abort requested: stopping the dome, restoring settings'
        return response

    def _on_heartbeat(self):
        # Keeps the plugin's bench heartbeat alive by re-sending the current percent;
        # once stopping it sends 0 instead (DriveGate).
        if not self._sweep_active:
            return
        sent = self._gate.heartbeat()
        if sent is not None and not sent[0] and not self._gate.stopping:
            self.request_abort('heartbeat drive call failed: %s' % sent[1])

    def _publish_status(self):
        with self._lock:
            status = dict(self._status)
            last = self._last_result
        started = status.pop('started_mono', None)
        step_started = status.pop('step_started_mono', None)
        now = time.monotonic()
        if started is not None:
            status['elapsed_s'] = round(now - started, 2)
        if step_started is not None:
            status['step_elapsed_s'] = round(now - step_started, 2)
        status['abort_requested'] = (self._abort_event.is_set() and
                                     status.get('state') != 'idle')
        status['last_result'] = last
        msg = String()
        msg.data = json.dumps(analysis.to_jsonable(status), allow_nan=False)
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Public control (signals)
    # ------------------------------------------------------------------

    def request_abort(self, reason):
        if not self._abort_event.is_set():
            self._abort_reason = reason
            self.get_logger().warn('Abort requested: %s' % reason)
        self._gate.begin_stopping()   # no non-zero percent from here on
        self._abort_event.set()

    def sweep_running(self):
        return self._claim.held

    def wait_for_worker(self):
        """Block until the worker has finished cleanup or a forced exit is requested."""
        while self.sweep_running() and not self._force_event.is_set():
            time.sleep(0.1)

    def force_exit(self):
        """Second signal: best-effort stop without waiting for the settings restore."""
        self._force_event.set()
        try:
            self._gate.force_stop(lock_timeout=0.5, timeout=0.5)
        except Exception:  # noqa: B902 - best effort on the way out
            pass
        self.get_logger().error(
            'Forced exit: the dome stops through the plugin heartbeat; settings may not be '
            'restored, see settings_before.json in the run directory')

    # ------------------------------------------------------------------
    # Service helpers (worker thread)
    # ------------------------------------------------------------------

    def _wait_future(self, client, future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            return None
        if future.exception() is not None:
            return None
        return future.result()

    def _call_drive(self, percent, timeout=SERVICE_TIMEOUT_S):
        """One bench/drive call; only DriveGate calls this."""
        if not self._drive_client.service_is_ready():
            return False, '%s/bench/drive is not available' % self._hw
        request = DomeBenchDrive.Request()
        request.percent = int(percent)
        response = self._wait_future(self._drive_client,
                                     self._drive_client.call_async(request), timeout)
        if response is None:
            return False, 'no response from %s/bench/drive within %.1f s' % (self._hw, timeout)
        return bool(response.success), response.message

    def _call_move(self, target_deg, timeout=SERVICE_TIMEOUT_S):
        """One bench/move call; only DriveGate calls this."""
        if not self._move_client.service_is_ready():
            return False, '%s/bench/move is not available' % self._hw
        request = DomeBenchMove.Request()
        request.target_deg = int(target_deg) % 360
        response = self._wait_future(self._move_client,
                                     self._move_client.call_async(request), timeout)
        if response is None:
            return False, 'no response from %s/bench/move within %.1f s' % (self._hw, timeout)
        return bool(response.success), response.message

    def _send_percent(self, percent):
        """Command a percent; refused for non-zero once the sweep is stopping."""
        return self._gate.send(percent)

    def _drive(self, percent, settings_retry_s=0.0):
        """Command a percent; a rejection aborts the sweep (plan: treat as abort)."""
        deadline = time.monotonic() + settings_retry_s
        while True:
            ok, message = self._send_percent(percent)
            if ok:
                return
            if ('settings write is in flight' in message and time.monotonic() < deadline and
                    not self._abort_event.is_set()):
                time.sleep(0.2)  # the raw_motor writes are still being acked
                continue
            raise SweepAbort('bench drive %d%% rejected: %s' % (percent, message))

    def _move(self, target_deg, settings_retry_s=0.0):
        """Command a bench position move; a rejection aborts the sweep."""
        deadline = time.monotonic() + settings_retry_s
        while True:
            ok, message = self._gate.move(target_deg)
            if ok:
                return
            if ('settings write is in flight' in message and time.monotonic() < deadline and
                    not self._abort_event.is_set()):
                time.sleep(0.2)
                continue
            raise SweepAbort('bench move to %d deg rejected: %s' % (target_deg, message))

    def _get_params(self, names):
        request = GetParameters.Request()
        request.names = ['device.' + n for n in names]
        if not self._get_client.service_is_ready():
            raise RuntimeError('%s/get_parameters is not available' % self._hw)
        response = self._wait_future(self._get_client, self._get_client.call_async(request),
                                     SERVICE_TIMEOUT_S)
        if response is None or len(response.values) != len(names):
            raise RuntimeError('could not read %s from %s' % (', '.join(names), self._hw))
        values = {}
        for name, value in zip(names, response.values):
            if value.type == 0:
                raise RuntimeError('%s has no parameter device.%s' % (self._hw, name))
            values[name] = parameter_value_to_python(value)
        return values

    def _set_param(self, name, value):
        request = SetParameters.Request()
        request.parameters = [Parameter('device.' + name, value=value).to_parameter_msg()]
        if not self._set_client.service_is_ready():
            return False, '%s/set_parameters is not available' % self._hw
        response = self._wait_future(self._set_client, self._set_client.call_async(request),
                                     SERVICE_TIMEOUT_S)
        if response is None or not response.results:
            return False, 'no response from %s/set_parameters' % self._hw
        return bool(response.results[0].successful), response.results[0].reason

    def _write_setting(self, name, value, interruptible):
        """Write one device setting, retrying while the plugin refuses it, then read it back."""
        deadline = time.monotonic() + SETTING_RETRY_S
        while True:
            ok, reason = self._set_param(name, value)
            if ok:
                break
            retry = any(r in reason for r in RETRYABLE_SET_REASONS)
            if not retry or time.monotonic() > deadline or self._force_event.is_set():
                raise RuntimeError('setting device.%s=%r rejected: %s' % (name, value, reason))
            if interruptible and self._abort_event.is_set():
                raise SweepAbort(self._abort_reason or 'abort')
            time.sleep(0.5)
        deadline = time.monotonic() + SETTING_READBACK_TIMEOUT_S
        while True:
            current = self._get_params([name])[name]
            if _same_value(current, value):
                self.get_logger().info('device.%s = %r (read back)' % (name, value))
                return
            if time.monotonic() > deadline:
                raise RuntimeError('device.%s reads back %r, expected %r' % (name, current, value))
            time.sleep(0.2)

    def _settings_counters(self, wait_s=0.0):
        """
        Latest (pending, acks, timeouts) from the plugin's diagnostics, or None.

        Only a status stamped within DIAGNOSTICS_WAIT_S counts, so counters left
        over from a hardware node that has since gone away are not used.
        """
        deadline = time.monotonic() + wait_s
        while True:
            with self._lock:
                fresh = time.time() - self._motion_stamp < DIAGNOSTICS_WAIT_S
                counters = self._motion_counters if fresh else None
            if counters is not None or time.monotonic() >= deadline:
                return counters
            time.sleep(0.1)

    def _wait_settings_acked(self, baseline, writes, what, issued_wall):
        """
        Wait until the firmware acked ``writes`` settings writes issued after ``baseline``.

        Uses the settings counters the plugin publishes on /diagnostics (roamadome:
        motion). Returns ``(acked, timeouts)``: ``acked`` is True when nothing is
        pending and the ack count advanced by ``writes`` with no ack timeout, False
        on a timeout or after SETTINGS_ACK_TIMEOUT_S, None when the plugin does not
        publish the counters. Only diagnostics stamped after ``issued_wall`` (wall
        time once the last write was accepted) count, so a message built before the
        writes were queued cannot report them as done.
        """
        if baseline is None:
            self.get_logger().warn(
                'No settings counters on /diagnostics (older plugin?); cannot confirm that '
                'the firmware acked the %s' % what)
            return None, None
        deadline = time.monotonic() + SETTINGS_ACK_TIMEOUT_S
        state, current = 'waiting', baseline
        while time.monotonic() < deadline and not self._force_event.is_set():
            with self._lock:
                fresh = self._motion_stamp > issued_wall
                if fresh:
                    current = self._motion_counters
            if fresh:
                state = control.settings_ack_state(baseline, current, writes)
                if state != 'waiting':
                    break
            time.sleep(0.1)
        timeouts = current[2] - baseline[2]
        if state == 'acked':
            self.get_logger().info('Firmware acked the %s (%d write(s))' % (what, writes))
            return True, timeouts
        self.get_logger().error(
            'Firmware did not ack the %s: %s (pending %d, acks +%d of %d, ack timeouts +%d)'
            % (what, 'ack timed out' if state == 'timeout' else 'no ack within %.0f s'
               % SETTINGS_ACK_TIMEOUT_S, current[0], current[1] - baseline[1], writes, timeouts))
        return False, timeouts

    def _apply_settings(self, desired, interruptible):
        """Write each setting that differs (writes persist to flash: write once)."""
        names = [n for n, _ in desired]
        current = self._get_params(names)
        written = {}
        for name, value in desired:
            if _same_value(current[name], value):
                continue
            self._write_setting(name, value, interruptible)
            written[name] = value
        return written

    # ------------------------------------------------------------------
    # Worker
    # ------------------------------------------------------------------

    def _read_sweep_params(self, sweep):
        def get(name):
            return self.get_parameter(name).value

        errors = control.validate_config(get)
        if errors:
            raise ValueError('invalid parameters: ' + '; '.join(errors))
        params = {
            'output_dir': os.path.expanduser(str(get('output_dir'))),
            # The position sweep measures the firmware's own moves at the operating settings.
            'raw_motor': bool(get('speed_sweep.raw_motor')) if sweep != 'position' else False,
            'heartbeat_period_s': self._heartbeat_period,
            'velocity_window_s': float(get('velocity_window_s')),
            'stall_deg_s': float(get('speed_sweep.stall_deg_s')),
            'stall_s': float(get('speed_sweep.stall_s')),
            'max_plausible_deg_s': float(get('sensor_sweep.max_plausible_deg_s')),
            'max_plausible_source': 'parameter sensor_sweep.max_plausible_deg_s',
            'wrap_allowance_deg': float(get('sensor_sweep.wrap_allowance_deg')),
            'same_cycle_dt_s': analysis.SAME_CYCLE_DT_S,
        }
        if sweep == 'sensor':
            params['percents'] = [int(p) for p in get('sensor_sweep.percents')]
            params['settle_s'] = float(get('sensor_sweep.settle_s'))
            params['duration_s'] = float(get('sensor_sweep.duration_s'))
        elif sweep == 'speed':
            params['percents'] = [int(p) for p in get('speed_sweep.percents')]
            params['settle_s'] = float(get('speed_sweep.settle_s'))
            params['min_measure_s'] = float(get('speed_sweep.min_measure_s'))
            params['max_measure_s'] = float(get('speed_sweep.max_measure_s'))
        else:
            params['stop_percents'] = [int(p) for p in get('position_sweep.stop_percents')]
            params['stop_run_s'] = float(get('position_sweep.stop_run_s'))
            params['moves_deg'] = [int(d) for d in get('position_sweep.moves_deg')]
            params['settle_s'] = float(get('position_sweep.settle_s'))
            params['move_timeout_s'] = float(get('position_sweep.move_timeout_s'))
            params['stopped_deg_s'] = float(get('position_sweep.stopped_deg_s'))
        return params

    def _make_run_dir(self, output_dir, sweep):
        os.makedirs(output_dir, exist_ok=True)
        base = os.path.join(output_dir, time.strftime('%Y%m%d-%H%M%S') + '-' + sweep)
        run_dir = base
        n = 1
        while os.path.exists(run_dir):
            n += 1
            run_dir = '%s-%d' % (base, n)
        os.makedirs(run_dir)
        return run_dir

    def _set_status(self, **fields):
        with self._lock:
            self._status.update(fields)

    def _latest(self):
        with self._lock:
            return self._latest_msg, self._latest_rx

    def _check_link(self):
        msg, rx = self._latest()
        age = time.monotonic() - rx
        if age > TELEMETRY_TIMEOUT_S:
            raise SweepAbort('telemetry dropout: no message for %.2f s' % age)
        if msg is not None and msg.command_mode not in (DomeTelemetry.COMMAND_MODE_IDLE,
                                                        DomeTelemetry.COMMAND_MODE_BENCH):
            raise SweepAbort('a controller took the dome (command_mode %d)' % msg.command_mode)

    def _check_abort(self):
        if self._abort_event.is_set():
            raise SweepAbort(self._abort_reason or 'abort requested')

    def _drain(self):
        """Move queued telemetry into the current step (or discard it between steps)."""
        batch = []
        while True:
            try:
                msg = self._telemetry_queue.get_nowait()
            except queue.Empty:
                break
            step = self._step
            if step is None:
                continue
            t = _stamp_seconds(msg.header.stamp)
            sample = analysis.Sample(
                t_host=t, seq=int(msg.sequence), position_deg=int(msg.position_deg),
                unwrapped_rad=float(msg.unwrapped_rad), velocity_rad_s=float(msg.velocity_rad_s),
                commanded_percent=int(msg.commanded_percent), direction=step['direction'],
                step_index=step['index'], phase=step['phase'])
            batch.append(sample)
            if step['last'] is not None:
                last_t, last_deg = step['last']
                d = analysis.shortest_delta(last_deg, sample.position_deg)
                step['unwrapped'] += d
                if analysis.is_jump(d, t - last_t, self._params['max_plausible_deg_s'],
                                    analysis.SAME_CYCLE_DT_S,
                                    self._params.get('wrap_allowance_deg', 0.0),
                                    analysis.crosses_zero(last_deg, d)):
                    self._counts['jumps'] += 1
            else:
                step['unwrapped'] = float(sample.position_deg)
            step['last'] = (t, sample.position_deg)
            step['times'].append(t)
            step['unwrapped_list'].append(step['unwrapped'])
            if step['stall'].update(t, step['unwrapped']):
                step['stalled'] = True
        if batch and self._csv is not None:
            self._csv.write(batch)
            self._counts['samples'] += len(batch)
        return batch

    def _hold(self, duration_fn):
        """Run the current step phase until duration_fn() elapses; return 'done' or 'stalled'."""
        start = time.monotonic()
        while True:
            self._drain()
            self._check_abort()
            self._check_link()
            if self._step['stalled']:
                return 'stalled'
            if time.monotonic() - start >= duration_fn():
                return 'done'
            time.sleep(LOOP_PERIOD_S)

    def _wait_standstill(self, strict):
        """After a stop: wait until telemetry shows 0% and |velocity| below the stall threshold."""
        stall_rad_s = math.radians(self._params['stall_deg_s'])
        start = time.monotonic()
        while time.monotonic() - start < STANDSTILL_TIMEOUT_S:
            self._drain()
            if strict:
                self._check_abort()
                self._check_link()
            msg, rx = self._latest()
            if (msg is not None and rx > start and msg.commanded_percent == 0 and
                    math.isfinite(msg.velocity_rad_s) and abs(msg.velocity_rad_s) < stall_rad_s):
                return True
            time.sleep(LOOP_PERIOD_S)
        self.get_logger().warn('No standstill within %.0f s of the stop' % STANDSTILL_TIMEOUT_S)
        return False

    def _run_step(self, kind, percent_abs, direction, settle_s, measure_fn):
        """One step: drive, settle, measure, then stop and wait for standstill."""
        index = len(self._steps)
        percent = percent_abs * direction
        meta = {'index': index, 'kind': kind, 'percent': percent, 'direction': direction,
                'end_reason': 'incomplete'}
        self._steps.append(meta)
        self._write_run_json('running')  # so a SIGKILL leaves the step list on disk
        self.get_logger().info('Step %d (%s): %d%%, settle %.1f s'
                               % (index, kind, percent, settle_s))
        self._check_abort()
        first_drive = not self._driven_once
        self._driven_once = True
        self._drive(percent, SETTINGS_IN_FLIGHT_RETRY_S if first_drive else 0.0)
        # Record from the moment the drive is accepted, so time spent retrying a
        # drive refused for an in-flight settings write never counts as a stall.
        self._drain()
        self._step = {'index': index, 'direction': direction, 'phase': analysis.PHASE_SETTLE,
                      'last': None, 'unwrapped': 0.0, 'times': [], 'unwrapped_list': [],
                      'stalled': False,
                      'stall': analysis.StallDetector(self._params['stall_deg_s'],
                                                      self._params['stall_s'])}
        self._set_status(phase=analysis.PHASE_SETTLE, step_index=index, kind=kind,
                         percent=percent, direction=direction,
                         step_started_mono=time.monotonic())
        try:
            outcome = self._hold(lambda: settle_s)
            if outcome == 'done':
                self._drain()
                self._step['phase'] = analysis.PHASE_MEASURE
                duration = measure_fn(self._step)
                self._set_status(phase=analysis.PHASE_MEASURE, measure_s=round(duration, 2))
                outcome = self._hold(lambda: duration)
            meta['end_reason'] = 'completed' if outcome == 'done' else 'stalled'
            if outcome == 'stalled':
                self._counts['stalls'] += 1
                self.get_logger().warn('Step %d: %d%% stalled; stopping' % (index, percent))
        finally:
            self._drain()
            self._step = None
        self._set_status(phase='stopping')
        self._drive(0)
        self._wait_standstill(strict=True)
        self._counts['steps_done'] += 1
        self._write_run_json('running')
        return meta

    def _revolution_measure(self, step):
        times, values = step['times'], step['unwrapped_list']
        cut = times[-1] - SPEED_ESTIMATE_S if times else 0.0
        tail = [(t, u) for t, u in zip(times, values) if t >= cut]
        speed = analysis.regression_slope([t for t, _ in tail], [u for _, u in tail]) \
            if len(tail) >= 2 else float('nan')
        return analysis.measure_duration(speed, self._params['min_measure_s'],
                                         self._params['max_measure_s'])

    def _run_sensor_sweep(self):
        duration = self._params['duration_s']
        for percent in self._params['percents']:
            for direction in (1, -1):
                self._run_step('step', percent, direction, self._params['settle_s'],
                               lambda step: duration)

    def _run_speed_sweep(self):
        entries = {1: [], -1: []}
        for percent in self._params['percents']:
            for direction in (1, -1):
                meta = self._run_step('step', percent, direction, self._params['settle_s'],
                                      self._revolution_measure)
                entries[direction].append((percent, meta['end_reason'] == 'completed'))
        for direction in (1, -1):
            while True:
                probe = analysis.next_search_probe(entries[direction])
                if probe is None or any(p == probe for p, _ in entries[direction]):
                    break
                meta = self._run_step('search', probe, direction, self._params['settle_s'],
                                      self._revolution_measure)
                entries[direction].append((probe, meta['end_reason'] == 'completed'))

    # ------------------------------------------------------------------
    # Position sweep (README "Position-move tuning")
    # ------------------------------------------------------------------

    def _wait_still(self, still_s, max_s, strict=True):
        """
        Wait until the reported angle has not changed for ``still_s`` seconds.

        Keeps draining telemetry into the current step. Returns True at
        standstill, False after ``max_s``. Position-based, unlike
        ``_wait_standstill``, so it also ends a firmware move, during which
        ``commanded_percent`` stays 0.
        """
        start = time.monotonic()
        ref = None
        last_change = start
        while time.monotonic() - start < max_s:
            self._drain()
            if strict:
                self._check_abort()
                self._check_link()
            msg, rx = self._latest()
            if msg is not None and rx >= start:
                if ref is None or int(msg.position_deg) != ref:
                    ref = int(msg.position_deg)
                    last_change = time.monotonic()
                elif time.monotonic() - last_change >= still_s:
                    return True
            time.sleep(LOOP_PERIOD_S)
        return False

    def _motion_status_values(self, after_wall=None, wait_s=0.0):
        """Latest roamadome: motion values, optionally only one stamped after ``after_wall``."""
        deadline = time.monotonic() + wait_s
        while True:
            with self._lock:
                stamp = self._motion_stamp
                values = dict(self._motion_values)
            if after_wall is None or stamp > after_wall or time.monotonic() >= deadline:
                return values
            time.sleep(0.1)

    def _begin_step(self, meta, phase, direction):
        """Register a position-sweep step and start recording into it."""
        self._steps.append(meta)
        self._write_run_json('running')
        self._drain()
        self._step = {'index': meta['index'], 'direction': direction, 'phase': phase,
                      'last': None, 'unwrapped': 0.0, 'times': [], 'unwrapped_list': [],
                      'stalled': False,
                      'stall': analysis.StallDetector(self._params['stall_deg_s'],
                                                      self._params['stall_s'])}
        self._set_status(phase=phase, step_index=meta['index'], kind=meta['kind'],
                         percent=meta['percent'], direction=direction,
                         step_started_mono=time.monotonic())

    def _run_stop_step(self, percent):
        """Steady ``percent`` for stop_run_s, then 0, recording the coast to standstill."""
        p = self._params
        index = len(self._steps)
        direction = 1 if percent > 0 else -1
        meta = {'index': index, 'kind': 'stop', 'percent': percent, 'direction': direction,
                'end_reason': 'incomplete'}
        self.get_logger().info('Step %d (stop): %d%% for %.1f s, then 0'
                               % (index, percent, p['stop_run_s']))
        self._check_abort()
        first_drive = not self._driven_once
        self._driven_once = True
        self._drive(percent, SETTINGS_IN_FLIGHT_RETRY_S if first_drive else 0.0)
        self._begin_step(meta, analysis.PHASE_RUN, direction)
        try:
            outcome = self._hold(lambda: p['stop_run_s'])
            if outcome == 'stalled':
                self._counts['stalls'] += 1
                self.get_logger().warn('Step %d: %d%% stalled; stopping' % (index, percent))
            self._drain()
            self._step['phase'] = analysis.PHASE_COAST
            self._set_status(phase=analysis.PHASE_COAST)
            self._drive(0)
            meta['t_stop_wall'] = time.time()
            meta['settled'] = self._wait_still(p['settle_s'], STOP_SETTLE_TIMEOUT_S)
            meta['end_reason'] = 'completed' if outcome == 'done' else 'stalled'
        finally:
            self._drain()
            self._step = None
        self._wait_standstill(strict=True)
        self._counts['steps_done'] += 1
        self._write_run_json('running')
        return meta

    def _run_move_step(self, delta):
        """One firmware position move of ``delta`` degrees from the current angle."""
        p = self._params
        index = len(self._steps)
        direction = 1 if delta > 0 else -1
        self._drain()
        msg, _ = self._latest()
        if msg is None:
            raise SweepAbort('no telemetry before move %d' % index)
        start = int(msg.position_deg)
        target = (start + delta) % 360
        meta = {'index': index, 'kind': 'move', 'percent': 0, 'direction': direction,
                'start_deg': start, 'target_deg': target, 'delta_deg': delta,
                'end_reason': 'incomplete'}
        before = self._motion_status_values()
        self.get_logger().info('Step %d (move): %d -> %d deg (delta %+d)'
                               % (index, start, target, delta))
        self._check_abort()
        first_drive = not self._driven_once
        self._driven_once = True
        self._move(target, SETTINGS_IN_FLIGHT_RETRY_S if first_drive else 0.0)
        meta['t_move_wall'] = time.time()
        self._begin_step(meta, analysis.PHASE_MOVE, direction)
        try:
            settled = self._wait_still(p['settle_s'], p['move_timeout_s'])
            t_still = time.time()
            meta['settled'] = settled
            meta['end_reason'] = 'completed' if settled else 'timeout'
            if not settled:
                self.get_logger().warn('Step %d: no standstill within %.0f s of the move'
                                       % (index, p['move_timeout_s']))
        finally:
            self._drain()
            self._step = None
        # The firmware's own verdict (REACHED TARGET / position timeout) reaches the
        # 1 Hz motion status a little later.
        after = self._motion_status_values(after_wall=t_still, wait_s=MOTION_STATUS_WAIT_S)
        meta['firmware'] = control.move_outcome(before, after)
        self._set_status(phase='releasing')
        self._drive(0)    # releases the bench claim; the move is over
        self._wait_standstill(strict=True)
        self._counts['steps_done'] += 1
        self._write_run_json('running')
        return meta

    def _run_position_sweep(self):
        speed_target = self._settings_before.get('speed_target')
        over = [p for p in self._params['stop_percents']
                if isinstance(speed_target, int) and p > speed_target]
        if over:
            self.get_logger().warn(
                'Stop percents %s exceed device.speed_target (%d): the firmware clamps :DPR to '
                'SpeedTarget, so those steps run at the same speed as %d %%' % (
                    over, speed_target, speed_target))
        for percent in self._params['stop_percents']:
            for direction in (1, -1):
                self._run_stop_step(percent * direction)
        for delta in self._params['moves_deg']:
            self._run_move_step(delta)

    def _resolve_sensor_thresholds(self):
        params = self._params
        highest = max(params['percents'])
        candidates = sorted(
            glob.glob(os.path.join(params['output_dir'], '*',
                                   analysis.result_filename('speed'))),
            key=os.path.getmtime, reverse=True)
        for path in candidates:
            try:
                result = analysis.read_json(path)
            except (OSError, ValueError):
                continue
            found = analysis.sensor_thresholds_from_speed_result(
                result, params['raw_motor'], highest)
            if found:
                params['max_plausible_deg_s'] = found[0]
                params['max_plausible_source'] = '1.5 x measured maximum in %s' % path
                params['min_percent_known'] = found[1]
                params['min_percent_source'] = path
                return
        min_speed = self._settings_before.get('min_speed', 0)
        params['min_percent_known'] = {name: min_speed
                                       for name in analysis.DIRECTION_NAMES.values()}
        params['min_percent_source'] = 'firmware MinSpeed (device.min_speed)'

    def _write_run_json(self, status, reason=None):
        info = {
            'sweep': self._sweep,
            'run_id': os.path.basename(self._run_dir),
            'hardware_node': self._hw,
            'started': self._started,
            'finished': _now_iso() if status != 'running' else None,
            'status': status,
            'reason': reason,
            'config': {k: v for k, v in self._params.items() if k != 'output_dir'},
            'steps': self._steps,
            'settings_before': self._settings_before,
            'settings_context': self._settings_context,
            'settings_written': self._settings_written,
            'settings_restored': self._settings_restored,
            'raw_settings_firmware_acked': self._raw_settings_acked,
            'settings_after': self._settings_after,
            'sensor_diagnostics_before': self._diag_before,
            'sensor_diagnostics_after': self._diag_after,
            'counts': self._counts,
        }
        analysis.write_json(os.path.join(self._run_dir, analysis.RUN_JSON), info)

    def _save_settings_before(self):
        saved = self._get_params(SAVED_SETTINGS)
        context = self._get_params(CONTEXT_SETTINGS)
        restore = ['ros2 param set %s device.%s %s'
                   % (self._hw, k, str(v).lower() if isinstance(v, bool) else v)
                   for k, v in saved.items() if k in dict(RAW_MOTOR_SETTINGS)]
        analysis.write_json(os.path.join(self._run_dir, analysis.SETTINGS_BEFORE_JSON), {
            'hardware_node': self._hw, 'time': _now_iso(), 'settings': saved,
            'context': context, 'restore_commands': restore})
        self._settings_before = saved
        self._settings_context = context
        self.get_logger().info('Saved settings_before.json: %s' % saved)

    def _sweep_main(self, sweep, params, run_dir):
        self._sweep = sweep
        self._params = params
        self._run_dir = run_dir
        self._started = _now_iso()
        self._steps = []
        self._step = None
        self._driven_once = False
        self._settings_before = {}
        self._settings_context = {}
        self._settings_written = {}
        self._settings_restored = None
        self._settings_after = None
        self._raw_settings_acked = None
        self._counts = {'samples': 0, 'jumps': 0, 'stalls': 0, 'steps_done': 0}
        with self._lock:
            self._diag_before = self._sensor_diag
        self._diag_after = None
        self._csv = None
        self._set_status(counts=self._counts)
        status, reason = 'error', None
        try:
            self._csv = analysis.CsvSampleWriter(os.path.join(run_dir, analysis.SAMPLES_CSV))
            self._set_status(phase='settings')
            self._save_settings_before()
            if sweep == 'sensor':
                self._resolve_sensor_thresholds()
            elif sweep == 'speed':
                min_speed = self._settings_before.get('min_speed', 0)
                params['min_percent_known'] = {n: min_speed
                                               for n in analysis.DIRECTION_NAMES.values()}
                params['min_percent_source'] = 'firmware MinSpeed (device.min_speed)'
            else:
                fudge = self._settings_context.get('fudge')
                params['fudge_deg'] = int(fudge) if isinstance(fudge, int) else None
            self._write_run_json('running')
            self._check_abort()
            if params['raw_motor']:
                baseline = self._settings_counters(DIAGNOSTICS_WAIT_S)
                self._settings_written = self._apply_settings(RAW_MOTOR_SETTINGS, True)
                issued = time.time()
                self._set_status(phase='waiting for firmware settings ack')
                acked, _ = self._wait_settings_acked(
                    baseline, len(self._settings_written), 'raw_motor settings', issued)
                self._raw_settings_acked = acked
                if acked is False:
                    raise RuntimeError('raw_motor settings not confirmed by the firmware')
            self._check_abort()
            if sweep == 'sensor':
                self._run_sensor_sweep()
            elif sweep == 'speed':
                self._run_speed_sweep()
            else:
                self._run_position_sweep()
            status = 'complete'
        except SweepAbort as exc:
            status, reason = 'aborted', str(exc)
            self.get_logger().warn('Sweep aborted: %s' % reason)
        except Exception as exc:  # noqa: B902 - any failure must still stop the dome
            status, reason = 'error', '%s: %s' % (type(exc).__name__, exc)
            self.get_logger().error('Sweep failed: %s\n%s' % (reason, traceback.format_exc()))
        finally:
            try:
                self._cleanup(status, reason)
            finally:
                self._claim.release()

    def _cleanup(self, status, reason):
        """Stop, wait for standstill, restore settings, write results. Never raises."""
        # Arm the stop on every path (completion, abort, exception): from here on the
        # gate refuses non-zero percents and the heartbeat only ever sends 0.
        self._gate.begin_stopping()
        self._step = None
        self._set_status(state='stopping', phase='stopping')
        stopped = False
        message = ''
        for _ in range(3):
            ok, message = self._send_percent(0)
            if ok:
                stopped = True
                break
            self.get_logger().error('Stop call failed: %s' % message)
            time.sleep(0.2)
        if not stopped:
            self.get_logger().error('Could not send percent 0 (%s): %s'
                                    % (message, control.stop_failure_advice(message)))
        self._wait_standstill(strict=False)
        with self._lock:
            self._sweep_active = False

        if self._settings_before:
            self._set_status(phase='restoring settings')
            desired = [(k, self._settings_before[k]) for k, _ in RAW_MOTOR_SETTINGS
                       if k in self._settings_before]
            try:
                baseline = self._settings_counters(DIAGNOSTICS_WAIT_S)
                restored = self._apply_settings(desired, interruptible=False)
                issued = time.time()
                if restored:
                    self.get_logger().info('Restore writes issued: %s' % restored)
                acked, timeouts = self._wait_settings_acked(
                    baseline, len(restored), 'settings restore', issued)
                if restored:
                    time.sleep(SETTINGS_SETTLE_S)  # post-ack #DPCONFIG refresh
                self._settings_after = self._get_params(SAVED_SETTINGS)
                ros_matches = all(_same_value(self._settings_after[k], v)
                                  for k, v in self._settings_before.items())
                self._settings_restored = ros_matches and acked is not False
                analysis.write_json(os.path.join(self._run_dir, analysis.SETTINGS_AFTER_JSON), {
                    'hardware_node': self._hw, 'time': _now_iso(),
                    'settings': self._settings_after,
                    'restore_writes': restored,
                    'ros_matches_before': ros_matches,
                    'firmware_acked': acked,
                    'ack_timeouts_during_restore': timeouts})
                if acked is False or not ros_matches:
                    self.get_logger().error(
                        'settings restore NOT confirmed by firmware; check device.* and %s'
                        % os.path.join(self._run_dir, analysis.SETTINGS_BEFORE_JSON))
                    if status == 'complete':
                        status, reason = 'error', 'settings restore not confirmed'
            except Exception as exc:  # noqa: B902 - report and carry on writing results
                self._settings_restored = False
                self.get_logger().error(
                    'Settings NOT restored (%s). Restore by hand from %s' % (
                        exc, os.path.join(self._run_dir, analysis.SETTINGS_BEFORE_JSON)))
                if status == 'complete':
                    status, reason = 'error', 'settings restore failed: %s' % exc
        with self._lock:
            self._diag_after = self._sensor_diag

        result_path = None
        try:
            if self._csv is not None:
                self._csv.close()
            self._write_run_json(status, reason)
            result = analysis.analyze_run_dir(self._run_dir)
            result_path = os.path.join(self._run_dir, analysis.result_filename(self._sweep))
            analysis.write_json(result_path, result)
            self.get_logger().info(
                '%s sweep %s; results in %s' % (self._sweep, status, result_path))
        except Exception as exc:  # noqa: B902
            self.get_logger().error('Writing results failed: %s\n%s'
                                    % (exc, traceback.format_exc()))
        with self._lock:
            self._last_result = {'sweep': self._sweep, 'status': status, 'reason': reason,
                                 'run_dir': self._run_dir, 'result': result_path,
                                 'settings_restored': self._settings_restored}
            self._status = {'state': 'idle'}


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = BenchNode()
    except ValueError as exc:
        get_logger('roamadome_bench').fatal(str(exc))
        rclpy.try_shutdown()
        return 2
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name='executor', daemon=True)
    spin_thread.start()

    stop = threading.Event()
    signals = []

    def on_signal(signum, _frame):
        signals.append(signum)
        if len(signals) == 1:
            if node.sweep_running():
                node.request_abort('signal %s' % signal.Signals(signum).name)
                node.get_logger().warn('Stopping the dome and restoring settings before exit; '
                                       'signal again to exit without restoring')
            stop.set()
        else:
            node.force_exit()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        while not stop.wait(0.2):
            if not spin_thread.is_alive():
                break
        node.wait_for_worker()
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
