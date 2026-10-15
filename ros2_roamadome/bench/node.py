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
The ``roamadome_bench`` node: sensor sweep and speed sweep for the dome.

See docs/dome_diagnostics_plan.md sections 5.4, 6 (phase 5) and 7. The node
drives the dome only through the hardware node's ``bench/drive`` service,
reads ``telemetry`` and changes ``device.*`` parameters through the hardware
node's parameter services. The hardware node is found by name through the
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
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import analysis

# Settings the bench may change, saved to settings_before.json (plan 6, phase 5 step 1).
SAVED_SETTINGS = ('max_speed', 'min_speed', 'speed_target', 'scaling')
# Recorded for context only; never written (Inverted, HomePos, AutoSafety are off limits).
CONTEXT_SETTINGS = ('speed_home', 'acceleration_scale', 'deceleration_scale', 'timeout',
                    'inverted', 'home_pos', 'auto_safety')
# raw_motor: max speed 100, target speed 100, scaling off (plan 5.4).
RAW_MOTOR_SETTINGS = (('max_speed', 100), ('speed_target', 100), ('scaling', False))

TELEMETRY_TIMEOUT_S = 1.0      # dropout during a sweep aborts it
STANDSTILL_TIMEOUT_S = 5.0     # plan: wait for standstill or 5 s
SERVICE_TIMEOUT_S = 2.0        # any single service call
DRIVE_LOCK_TIMEOUT_S = 3.0
SETTING_READBACK_TIMEOUT_S = 5.0
SETTING_RETRY_S = 10.0         # retry a write the plugin refuses while the dome is stopping
SETTINGS_IN_FLIGHT_RETRY_S = 12.0  # first drive after a settings write waits for its ack
SETTINGS_SETTLE_S = 2.0        # let the post-ack #DPCONFIG refresh land before reading back
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
        self.declare_parameter('speed_sweep.percents',
                               [10, 15, 20, 25, 30, 40, 50, 60, 70, 80, 90, 100])
        self.declare_parameter('speed_sweep.settle_s', 2.0)
        self.declare_parameter('speed_sweep.min_measure_s', 4.0)
        self.declare_parameter('speed_sweep.max_measure_s', 30.0)
        self.declare_parameter('speed_sweep.stall_deg_s', 3.0)
        self.declare_parameter('speed_sweep.stall_s', 2.0)
        self.declare_parameter('speed_sweep.raw_motor', True)

        hw = str(self.get_parameter('hardware_node').value).strip().rstrip('/')
        self._hw = hw if hw.startswith('/') else '/' + hw

        self._lock = threading.Lock()          # status, latest telemetry, worker handle
        self._drive_lock = threading.Lock()    # serialises every bench/drive call
        self._abort_event = threading.Event()
        self._force_event = threading.Event()  # second signal: stop waiting for cleanup
        self._abort_reason = None
        self._worker = None
        self._sweep_active = False
        self._current_percent = 0
        self._telemetry_queue = queue.Queue()
        self._latest_msg = None
        self._latest_rx = 0.0
        self._sensor_diag = None
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

    def _on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'roamadome: sensor':
                values = {kv.key: kv.value for kv in status.values}
                values['level'] = int.from_bytes(status.level, 'little') \
                    if isinstance(status.level, bytes) else int(status.level)
                values['message'] = status.message
                with self._lock:
                    self._sensor_diag = values

    def _on_start(self, sweep, response):
        with self._lock:
            busy = self._worker is not None and self._worker.is_alive()
            latest = self._latest_msg
            age = time.monotonic() - self._latest_rx
        response.success = False
        if busy:
            response.message = 'a sweep is already running'
            return response
        if latest is None or age > TELEMETRY_TIMEOUT_S:
            response.message = 'no telemetry on %s/telemetry (is publish_telemetry on?)' % self._hw
            return response
        if latest.command_mode != DomeTelemetry.COMMAND_MODE_IDLE or latest.commanded_percent:
            response.message = ('hardware is not idle (command_mode %d, commanded %d%%); a '
                                'controller or another bench client holds the dome'
                                % (latest.command_mode, latest.commanded_percent))
            return response
        for client, name in ((self._drive_client, 'bench/drive'),
                             (self._get_client, 'get_parameters'),
                             (self._set_client, 'set_parameters')):
            if not client.service_is_ready():
                response.message = '%s/%s is not available (bench mode off?)' % (self._hw, name)
                return response
        try:
            params = self._read_sweep_params(sweep)
            run_dir = self._make_run_dir(params['output_dir'], sweep)
        except (ValueError, OSError) as exc:
            response.message = str(exc)
            return response

        self._abort_event.clear()
        self._abort_reason = None
        while not self._telemetry_queue.empty():
            self._telemetry_queue.get_nowait()
        worker = threading.Thread(target=self._sweep_main, args=(sweep, params, run_dir),
                                  name='roamadome_bench_%s' % sweep, daemon=True)
        with self._lock:
            self._worker = worker
            self._sweep_active = True
            self._status = {'state': 'running', 'sweep': sweep, 'run_dir': run_dir,
                            'phase': 'starting', 'started_mono': time.monotonic()}
        worker.start()
        response.success = True
        response.message = '%s sweep started; results in %s' % (sweep, run_dir)
        self.get_logger().info(response.message)
        return response

    def _on_abort(self, request, response):
        with self._lock:
            running = self._worker is not None and self._worker.is_alive()
        if not running:
            response.success = False
            response.message = 'no sweep is running'
            return response
        self.request_abort('abort service')
        response.success = True
        response.message = 'abort requested: stopping the dome, restoring settings'
        return response

    def _on_heartbeat(self):
        # Keeps the plugin's bench heartbeat alive by re-sending the current percent.
        if not self._sweep_active:
            return
        if not self._drive_lock.acquire(blocking=False):
            return  # the worker is sending right now, which refreshes the heartbeat
        try:
            if self._abort_event.is_set():
                if self._current_percent != 0:
                    self._current_percent = 0
                    self._call_drive_locked(0)
                return
            percent = self._current_percent
            if percent == 0:
                return
            ok, message = self._call_drive_locked(percent)
            if not ok:
                self._current_percent = 0
                self.request_abort('heartbeat drive call failed: %s' % message)
        finally:
            self._drive_lock.release()

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
        self._abort_event.set()

    def sweep_running(self):
        with self._lock:
            return self._worker is not None and self._worker.is_alive()

    def wait_for_worker(self):
        """Block until the worker has finished cleanup or a forced exit is requested."""
        while self.sweep_running() and not self._force_event.is_set():
            time.sleep(0.1)

    def force_exit(self):
        """Second signal: best-effort stop without waiting for the settings restore."""
        self._force_event.set()
        self._current_percent = 0
        try:
            self._call_drive_locked(0, timeout=0.5)
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

    def _call_drive_locked(self, percent, timeout=SERVICE_TIMEOUT_S):
        if not self._drive_client.service_is_ready():
            return False, '%s/bench/drive is not available' % self._hw
        request = DomeBenchDrive.Request()
        request.percent = int(percent)
        response = self._wait_future(self._drive_client,
                                     self._drive_client.call_async(request), timeout)
        if response is None:
            return False, 'no response from %s/bench/drive within %.1f s' % (self._hw, timeout)
        return bool(response.success), response.message

    def _send_percent(self, percent):
        if not self._drive_lock.acquire(timeout=DRIVE_LOCK_TIMEOUT_S):
            return False, 'drive lock timeout'
        try:
            self._current_percent = int(percent)
            ok, message = self._call_drive_locked(percent)
            if not ok:
                self._current_percent = 0
            return ok, message
        finally:
            self._drive_lock.release()

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

        prefix = 'sensor_sweep.' if sweep == 'sensor' else 'speed_sweep.'
        percents = sorted({abs(int(p)) for p in (get(prefix + 'percents') or [])})
        percents = [p for p in percents if 0 < p <= 100]
        if not percents:
            raise ValueError('%spercents has no value in 1..100' % prefix)
        params = {
            'output_dir': os.path.expanduser(str(get('output_dir'))),
            'percents': percents,
            'raw_motor': bool(get('speed_sweep.raw_motor')),
            'heartbeat_period_s': self._heartbeat_period,
            'velocity_window_s': float(get('velocity_window_s')),
            'stall_deg_s': float(get('speed_sweep.stall_deg_s')),
            'stall_s': float(get('speed_sweep.stall_s')),
            'max_plausible_deg_s': float(get('sensor_sweep.max_plausible_deg_s')),
            'max_plausible_source': 'parameter sensor_sweep.max_plausible_deg_s',
            'same_cycle_dt_s': analysis.SAME_CYCLE_DT_S,
        }
        if sweep == 'sensor':
            params['settle_s'] = float(get('sensor_sweep.settle_s'))
            params['duration_s'] = float(get('sensor_sweep.duration_s'))
        else:
            params['settle_s'] = float(get('speed_sweep.settle_s'))
            params['min_measure_s'] = float(get('speed_sweep.min_measure_s'))
            params['max_measure_s'] = float(get('speed_sweep.max_measure_s'))
        for key in ('velocity_window_s', 'stall_deg_s', 'stall_s', 'max_plausible_deg_s'):
            if not params[key] > 0.0:
                raise ValueError('%s must be > 0' % key)
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
                if analysis.is_jump(d, t - last_t, self._params['max_plausible_deg_s']):
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
            'settings_written': self._settings_written,
            'settings_restored': self._settings_restored,
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
        self._settings_written = {}
        self._settings_restored = None
        self._settings_after = None
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
            else:
                min_speed = self._settings_before.get('min_speed', 0)
                params['min_percent_known'] = {n: min_speed
                                               for n in analysis.DIRECTION_NAMES.values()}
                params['min_percent_source'] = 'firmware MinSpeed (device.min_speed)'
            self._write_run_json('running')
            self._check_abort()
            if params['raw_motor']:
                self._settings_written = self._apply_settings(RAW_MOTOR_SETTINGS, True)
            self._check_abort()
            if sweep == 'sensor':
                self._run_sensor_sweep()
            else:
                self._run_speed_sweep()
            status = 'complete'
        except SweepAbort as exc:
            status, reason = 'aborted', str(exc)
            self.get_logger().warn('Sweep aborted: %s' % reason)
        except Exception as exc:  # noqa: B902 - any failure must still stop the dome
            status, reason = 'error', '%s: %s' % (type(exc).__name__, exc)
            self.get_logger().error('Sweep failed: %s\n%s' % (reason, traceback.format_exc()))
        finally:
            self._cleanup(status, reason)

    def _cleanup(self, status, reason):
        """Stop, wait for standstill, restore settings, write results. Never raises."""
        self._step = None
        self._set_status(state='stopping', phase='stopping')
        stopped = False
        for _ in range(3):
            ok, message = self._send_percent(0)
            if ok:
                stopped = True
                break
            self.get_logger().error('Stop call failed: %s' % message)
            time.sleep(0.2)
        if not stopped:
            self.get_logger().error('Could not send percent 0; the plugin heartbeat stops the '
                                    'dome within its timeout')
        self._wait_standstill(strict=False)
        with self._lock:
            self._sweep_active = False

        if self._settings_before:
            self._set_status(phase='restoring')
            desired = [(k, self._settings_before[k]) for k, _ in RAW_MOTOR_SETTINGS
                       if k in self._settings_before]
            try:
                restored = self._apply_settings(desired, interruptible=False)
                self._settings_restored = True
                if restored:
                    self.get_logger().info('Restored settings: %s' % restored)
                    time.sleep(SETTINGS_SETTLE_S)
                self._settings_after = self._get_params(SAVED_SETTINGS)
                analysis.write_json(os.path.join(self._run_dir, analysis.SETTINGS_AFTER_JSON), {
                    'hardware_node': self._hw, 'time': _now_iso(),
                    'settings': self._settings_after,
                    'matches_before': all(_same_value(self._settings_after[k], v)
                                          for k, v in self._settings_before.items())})
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
    node = BenchNode()
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
