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
Control helpers for the roamadome_bench node, without ROS imports.

The safety-relevant pieces of the node live here so they can be unit tested
without a ROS graph: the atomic sweep claim, the drive gate that serialises
bench drive calls and refuses a non-zero percent once a sweep is stopping,
parameter validation, the firmware settings-ack check and the advice printed
when a stop call fails.
"""

import threading

HEARTBEAT_MAX_S = 1.0          # the plugin's bench watchdog defaults to 1.5 s
DRIVE_LOCK_TIMEOUT_S = 3.0
SERVICE_TIMEOUT_S = 2.0

REFUSED_STOPPING = 'the sweep is stopping: non-zero percent refused'
REFUSED_MOVE_ACTIVE = 'a bench move is being kept alive: release it with percent 0 first'
REFUSED_DRIVE_ACTIVE = 'a non-zero percent is being kept alive: send 0 before a move'


class SweepClaim:
    """At most one sweep at a time: claimed atomically, released by its owner."""

    def __init__(self):
        self._lock = threading.Lock()
        self._held = False

    def try_claim(self):
        """Claim the sweep slot; return False if a sweep already holds it."""
        with self._lock:
            if self._held:
                return False
            self._held = True
            return True

    def release(self):
        with self._lock:
            self._held = False

    @property
    def held(self):
        with self._lock:
            return self._held


class DriveGate:
    """
    Serialise every bench drive call and remember what to keep alive.

    ``call(percent, timeout)`` performs one bench/drive call and returns
    ``(ok, message)``; ``call_move(target_deg, speed_percent, timeout)`` performs one
    bench/move call (optional). The gate remembers the percent, or the move
    target, that the heartbeat must repeat. Once :meth:`begin_stopping` has been
    called (abort, cleanup, signal) only percent 0 is sent until :meth:`arm`
    starts the next sweep, so neither the worker nor the heartbeat can restart
    the dome. Percent 0 also releases a move claim (the firmware finishes a
    move on its own; the host cannot cancel it).
    """

    def __init__(self, call, call_move=None):
        self._call = call
        self._call_move = call_move
        self._lock = threading.Lock()
        self._percent = 0
        self._move = None
        self._move_speed = 0
        self._stopping = threading.Event()

    def arm(self):
        """Start of a sweep: nothing commanded, non-zero percents allowed."""
        self._percent = 0
        self._move = None
        self._move_speed = 0
        self._stopping.clear()

    def begin_stopping(self):
        """From now on refuse non-zero percents; the heartbeat sends 0 if needed."""
        self._stopping.set()

    @property
    def stopping(self):
        return self._stopping.is_set()

    @property
    def percent(self):
        return self._percent

    @property
    def move_speed(self):
        """Return the speed percent of the move being kept alive (0 = default)."""
        return self._move_speed

    @property
    def move_target(self):
        """Return the bench move target being kept alive, or None."""
        return self._move

    def send(self, percent, lock_timeout=DRIVE_LOCK_TIMEOUT_S, timeout=SERVICE_TIMEOUT_S):
        """Command ``percent`` (worker). Returns ``(ok, message)``."""
        percent = int(percent)
        if percent != 0 and self._stopping.is_set():
            return False, REFUSED_STOPPING
        if not self._lock.acquire(timeout=lock_timeout):
            return False, 'drive lock timeout'
        try:
            if percent != 0 and self._stopping.is_set():
                return False, REFUSED_STOPPING
            if percent != 0 and self._move is not None:
                return False, REFUSED_MOVE_ACTIVE
            self._percent = percent
            if percent == 0:
                self._move = None   # releases the move claim
                self._move_speed = 0
            ok, message = self._call(percent, timeout)
            if not ok:
                self._percent = 0
            return ok, message
        finally:
            self._lock.release()

    def move(self, target_deg, speed_percent=0, lock_timeout=DRIVE_LOCK_TIMEOUT_S,
             timeout=SERVICE_TIMEOUT_S):
        """
        Command a firmware position move; return ``(ok, message)``.

        ``speed_percent`` 0 keeps the firmware SpeedTarget (or the plugin default); 1..100
        is a per-move speed. The heartbeat repeats the same (target, speed) pair.
        """
        target_deg = int(target_deg) % 360
        speed_percent = int(speed_percent)
        if self._call_move is None:
            return False, 'bench move is not supported'
        if self._stopping.is_set():
            return False, REFUSED_STOPPING
        if not self._lock.acquire(timeout=lock_timeout):
            return False, 'drive lock timeout'
        try:
            if self._stopping.is_set():
                return False, REFUSED_STOPPING
            if self._percent != 0:
                return False, REFUSED_DRIVE_ACTIVE
            ok, message = self._call_move(target_deg, speed_percent, timeout)
            self._move = target_deg if ok else None
            self._move_speed = speed_percent if ok else 0
            return ok, message
        finally:
            self._lock.release()

    def heartbeat(self, timeout=SERVICE_TIMEOUT_S):
        """
        Re-send the current percent (heartbeat timer).

        Returns None when nothing was sent (the worker holds the lock, or the
        dome is not commanded), else ``(ok, message)``. While stopping it sends
        0 once if a non-zero percent is still recorded.
        """
        if not self._lock.acquire(blocking=False):
            return None  # the worker is sending, which refreshes the plugin heartbeat
        try:
            if self._stopping.is_set():
                if self._percent == 0 and self._move is None:
                    return None
                self._percent = 0
                self._move = None
                self._move_speed = 0
                return self._call(0, timeout)
            if self._move is not None:
                ok, message = self._call_move(self._move, self._move_speed, timeout)
                if not ok:
                    self._move = None
                    self._move_speed = 0
                return ok, message
            if self._percent == 0:
                return None
            ok, message = self._call(self._percent, timeout)
            if not ok:
                self._percent = 0
            return ok, message
        finally:
            self._lock.release()

    def force_stop(self, lock_timeout=0.5, timeout=0.5):
        """Last-resort 0 on exit: take the lock briefly if possible, send 0 regardless."""
        self._stopping.set()
        locked = self._lock.acquire(timeout=lock_timeout)
        try:
            self._percent = 0
            self._move = None
            self._move_speed = 0
            return self._call(0, timeout)
        finally:
            if locked:
                self._lock.release()


# ---------------------------------------------------------------------------
# Parameter validation
# ---------------------------------------------------------------------------

def validate_percents(name, values):
    """Errors for a percent list: integers in 1..100, strictly ascending, no duplicates."""
    errors = []
    values = list(values or [])
    if not values:
        return ['%s is empty' % name]
    for v in values:
        if isinstance(v, bool) or not isinstance(v, int) or not 1 <= v <= 100:
            errors.append('%s: %r is not an integer in 1..100' % (name, v))
    if len(set(values)) != len(values):
        errors.append('%s has duplicates: %s' % (name, values))
    elif any(b <= a for a, b in zip(values, values[1:])):
        errors.append('%s must be ascending (lowest first): %s' % (name, values))
    return errors


def validate_moves(name, values):
    """
    Errors for a move list: non-zero integers in -179..179, at least one.

    The firmware takes the short way round, so a move of 180 deg or more would
    not travel in the direction of its sign and the analysis could not score it.
    """
    errors = []
    values = list(values or [])
    if not values:
        return ['%s is empty' % name]
    for v in values:
        if isinstance(v, bool) or not isinstance(v, int) or v == 0 or not -179 <= v <= 179:
            errors.append('%s: %r is not a non-zero integer in -179..179' % (name, v))
    return errors


def validate_config(get):
    """
    Return the list of errors in the bench parameters; ``get(name)`` reads a value.

    Rejected: heartbeat_period_s outside (0, HEARTBEAT_MAX_S]; bad percent
    lists; negative settle times; non-positive durations and thresholds;
    min_measure_s greater than max_measure_s; bad move lists.
    """
    errors = []
    heartbeat = get('heartbeat_period_s')
    if not 0.0 < heartbeat <= HEARTBEAT_MAX_S:
        errors.append('heartbeat_period_s must be in (0, %.1f] s (the plugin watchdog is '
                      '1.5 s by default), got %r' % (HEARTBEAT_MAX_S, heartbeat))
    errors += validate_percents('sensor_sweep.percents', get('sensor_sweep.percents'))
    errors += validate_percents('speed_sweep.percents', get('speed_sweep.percents'))
    errors += validate_percents('position_sweep.stop_percents',
                                get('position_sweep.stop_percents'))
    errors += validate_moves('position_sweep.moves_deg', get('position_sweep.moves_deg'))
    speed = get('position_sweep.move_speed_percent')
    if isinstance(speed, bool) or not isinstance(speed, int) or not 0 <= speed <= 100:
        errors.append('position_sweep.move_speed_percent must be an integer in 0..100, got %r'
                      % (speed,))
    for name in ('sensor_sweep.settle_s', 'speed_sweep.settle_s',
                 'sensor_sweep.wrap_allowance_deg'):
        if get(name) < 0.0:
            errors.append('%s must be >= 0, got %r' % (name, get(name)))
    for name in ('sensor_sweep.duration_s', 'speed_sweep.min_measure_s',
                 'speed_sweep.max_measure_s', 'speed_sweep.stall_deg_s', 'speed_sweep.stall_s',
                 'sensor_sweep.max_plausible_deg_s', 'velocity_window_s',
                 'position_sweep.stop_run_s', 'position_sweep.settle_s',
                 'position_sweep.move_timeout_s', 'position_sweep.stopped_deg_s'):
        if not get(name) > 0.0:
            errors.append('%s must be > 0, got %r' % (name, get(name)))
    if get('speed_sweep.min_measure_s') > get('speed_sweep.max_measure_s'):
        errors.append('speed_sweep.min_measure_s (%r) must not exceed speed_sweep.max_measure_s '
                      '(%r)' % (get('speed_sweep.min_measure_s'),
                                get('speed_sweep.max_measure_s')))
    return errors


# ---------------------------------------------------------------------------
# Firmware settings acknowledgement (plugin /diagnostics, roamadome: motion)
# ---------------------------------------------------------------------------

def parse_settings_counters(values):
    """
    Extract the settings-write counters from a ``roamadome: motion`` status.

    ``values`` maps key to string. Returns ``(pending, acks, timeouts)`` or None
    if the plugin does not publish them (an older plugin).
    """
    try:
        return (int(values['settings_writes_pending']), int(values['settings_acks']),
                int(values['settings_ack_timeouts']))
    except (KeyError, TypeError, ValueError):
        return None


def settings_ack_state(baseline, current, writes):
    """
    Decide whether ``writes`` settings writes issued after ``baseline`` are acked.

    ``baseline`` and ``current`` are ``(pending, acks, timeouts)``. Returns
    ``'acked'`` when nothing is pending, the ack count advanced by at least
    ``writes`` (``Unchanged`` counts as an ack) and no ack timed out;
    ``'timeout'`` as soon as an ack timed out; otherwise ``'waiting'``.
    """
    _, base_acks, base_timeouts = baseline
    pending, acks, timeouts = current
    if timeouts != base_timeouts:
        return 'timeout'
    if pending == 0 and acks - base_acks >= writes:
        return 'acked'
    return 'waiting'


def move_outcome(before, after, fresh=True, move_age_s=None):
    """
    Return what the firmware reported for one bench move.

    ``before`` and ``after`` are the ``roamadome: motion`` value dicts before the
    move and after standstill; ``fresh`` says whether ``after`` was published after
    the standstill, and ``move_age_s`` how long before ``after`` the move was
    commanded. Returns ``position_timeouts`` (new firmware position timeouts
    during the move), ``reached_target_deg`` (the angle of a REACHED TARGET that
    completed during this move, else None) and ``status_seen`` (False when no
    fresh status arrived, in which case the other two are unknown and None).
    """
    def count(values, key):
        try:
            return int(values.get(key))
        except (TypeError, ValueError):
            return None

    def age(values):
        try:
            return float(values.get('last_move_completed_age_s'))
        except (TypeError, ValueError):
            return None

    if not after or not fresh:
        return {'status_seen': False, 'position_timeouts': None, 'reached_target_deg': None}
    timeouts_before = count(before or {}, 'position_timeouts')
    timeouts_after = count(after, 'position_timeouts')
    timeouts = None
    if timeouts_before is not None and timeouts_after is not None:
        timeouts = max(0, timeouts_after - timeouts_before)
    reached = None
    before_deg = (before or {}).get('last_target_reached_deg')
    after_deg = after.get('last_target_reached_deg')
    completed_age = age(after)
    # REACHED TARGET counts only if it completed during this move: the status is
    # published at 1 Hz, so allow one period of slack on the move's age.
    during_move = (completed_age is not None and move_age_s is not None and
                   0.0 <= completed_age <= move_age_s + 1.5)
    if after_deg not in (None, 'unknown') and (
            (after_deg != before_deg and (move_age_s is None or during_move)) or during_move):
        try:
            reached = int(after_deg)
        except ValueError:
            reached = None
    return {'status_seen': True, 'position_timeouts': timeouts, 'reached_target_deg': reached}


# ---------------------------------------------------------------------------
# Stop failures
# ---------------------------------------------------------------------------

def stop_failure_advice(message):
    """Explain what a failed percent-0 call means for the dome, from the plugin's reason."""
    if 'inactive' in message or 'controller is commanding' in message:
        return ('the dome was already stopped by the plugin (it sends :DPR0 on deactivation '
                'and when a controller takes over)')
    if 'serial port is closed' in message:
        return ('the plugin cannot reach the controller (serial port closed); check that the '
                'dome has stopped')
    return ('if the hardware node is alive its bench heartbeat stops the dome within '
            'bench_heartbeat_timeout_ms; if it died, check that the dome has stopped')
