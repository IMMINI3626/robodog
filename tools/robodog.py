import atexit
import signal
import serial
import binascii
import math
import sys
import time as _time_module
from collections import deque
from time import monotonic, sleep
import platform
import random
import weakref
from operator import eq
from queue import Queue
from threading import Event, Lock, RLock, Thread, current_thread, main_thread
import subprocess
from serial.tools.list_ports import comports
from .parse import *
from .packet import *
from .deflib import *


_ACTIVE_DOGS = weakref.WeakSet()
_ACTIVE_DOGS_LOCK = Lock()
_PREVIOUS_SIGINT_HANDLER = None
_SIGINT_HANDLER_INSTALLED = False
_ORIGINAL_TIME_SLEEP = _time_module.sleep


def _after_user_sleep():
    for dog in _snapshot_active_dogs():
        try:
            dog._maybe_stop_after_sleep()
        except BaseException:
            pass


def _robodog_sleep(seconds):
    _ORIGINAL_TIME_SLEEP(seconds)
    _after_user_sleep()


def _snapshot_active_dogs():
    with _ACTIVE_DOGS_LOCK:
        return list(_ACTIVE_DOGS)


def _track_active_dog(dog):
    with _ACTIVE_DOGS_LOCK:
        _ACTIVE_DOGS.add(dog)


def _untrack_active_dog(dog):
    with _ACTIVE_DOGS_LOCK:
        _ACTIVE_DOGS.discard(dog)


def _cleanup_active_dogs():
    for dog in _snapshot_active_dogs():
        try:
            dog._auto_cleanup_close()
        except BaseException:
            pass


def _sigint_cleanup_handler(signum, frame):
    _cleanup_active_dogs()
    previous = _PREVIOUS_SIGINT_HANDLER
    if previous in (None, signal.SIG_DFL):
        raise KeyboardInterrupt
    if previous == signal.SIG_IGN:
        return
    previous(signum, frame)


def _ensure_sigint_handler():
    global _PREVIOUS_SIGINT_HANDLER, _SIGINT_HANDLER_INSTALLED
    if current_thread() is not main_thread():
        return
    with _ACTIVE_DOGS_LOCK:
        if _SIGINT_HANDLER_INSTALLED:
            return
        current_handler = signal.getsignal(signal.SIGINT)
        if current_handler == signal.SIG_IGN:
            return
        _PREVIOUS_SIGINT_HANDLER = current_handler
        signal.signal(signal.SIGINT, _sigint_cleanup_handler)
        _SIGINT_HANDLER_INSTALLED = True


def _ensure_sleep_hook():
    if _time_module.sleep is not _robodog_sleep:
        _time_module.sleep = _robodog_sleep

    main_module = sys.modules.get("__main__")
    if main_module is not None and getattr(main_module, "sleep", None) is _ORIGINAL_TIME_SLEEP:
        setattr(main_module, "sleep", _robodog_sleep)


atexit.register(_cleanup_active_dogs)


def _decode_sound_track(track):
    track = int(track)
    return divmod(track, 10) if track >= 10 else (0, track)


def _resolve_sound_components(thema_or_track, track_or_volume, volume):
    if volume is None:
        thema, track = _decode_sound_track(thema_or_track)
        volume = track_or_volume
    else:
        thema = int(thema_or_track)
        track = int(track_or_volume)
        if track >= 10 and track // 10 == thema:
            thema, track = _decode_sound_track(track)
    return thema, track, volume


class RoboDog(Parse, Packet):
    def __init__(self, receiveCallback=None, index=0):
        super().__init__()
        self.serial = None
        self.isTXThreadRun = False
        self.isRXThreadRun = False
        self.parse = Parse(ROBODOG)
        self.makepkt = Packet(ROBODOG)
        self.receiveCallback = receiveCallback
        self.ledPacket = bytearray(34)
        self.rcvPacket = bytearray(24)
        self.portIndex = index
        self.txCnt = 0
        self.manualPacket = None
        self.txThread = None
        self.rxThread = None
        self._spin_turn = None
        self._spin_speed = 0
        self._spin_target = None
        self._spin_source_seq = -1
        self._move_with_bend_vel = 0
        self._move_with_bend_step = 0
        self._move_with_bend_heights = None
        self._move_with_bend_phase = 0
        self._move_with_bend_next_phase_at = 0.0
        self._sleep_bound_move = False
        self._sleep_bound_turn = False
        self._state_lock = RLock()
        self._rx_lock = Lock()
        self._close_lock = Lock()
        self._close_in_progress = False
        self._closed = True
        self._rx_ready = Event()
        self._rx_sequence = 0
        self._last_rx_time = None
        self._last_raw_rotation = None
        self._last_unwrapped_rotation = None
        self._rotation_samples = deque(maxlen=3)
        self._control_rotation = 0
        self._rx_ready_timeout = 0.5
        self._spin_stale_timeout = 0.5

    def _auto_cleanup_close(self):
        self.Close()

    def _safe_is_open(self):
        try:
            return self.serial is not None and self.serial.isOpen()
        except (AttributeError, OSError, TypeError):
            return False

    def _wait_for_rx_ready(self, timeout=None):
        if self._rx_ready.is_set():
            return True
        if not self._safe_is_open():
            return False
        wait_timeout = self._rx_ready_timeout if timeout is None else timeout
        return self._rx_ready.wait(wait_timeout)

    def _write_serial(self, packet):
        try:
            if self.serial is None:
                return False
            self.serial.write(packet)
            return True
        except (AttributeError, OSError, TypeError, serial.SerialException):
            return False

    def _read_serial(self):
        try:
            if self.serial is None:
                return None
            return self.serial.read(self.serial.in_waiting or 1)
        except (AttributeError, OSError, TypeError, serial.SerialException):
            return None

    def _update_packet(self, updater, recalculate_checksum=True):
        return self.makepkt.updatePacket(updater, recalculate_checksum=recalculate_checksum)

    @staticmethod
    def _wrap_delta(current_value, previous_value):
        delta = current_value - previous_value
        if delta > 32767:
            delta -= 65536
        elif delta < -32768:
            delta += 65536
        return delta

    def _reset_rx_state(self):
        with self._rx_lock:
            self.rcvPacket = bytearray(24)
            self._rx_sequence = 0
            self._last_rx_time = None
            self._last_raw_rotation = None
            self._last_unwrapped_rotation = None
            self._rotation_samples.clear()
            self._control_rotation = 0
        self._rx_ready.clear()

    def _update_rx_state(self, packet):
        raw_rotation = DefLib.toSigned16((packet[11] << 8) | packet[10])
        with self._rx_lock:
            copy_len = min(len(packet), len(self.rcvPacket))
            self.rcvPacket[:copy_len] = packet[:copy_len]
            if self._last_raw_rotation is None:
                unwrapped_rotation = raw_rotation
            else:
                delta = self._wrap_delta(raw_rotation, self._last_raw_rotation)
                unwrapped_rotation = self._last_unwrapped_rotation + delta
            self._last_raw_rotation = raw_rotation
            self._last_unwrapped_rotation = unwrapped_rotation
            self._rotation_samples.append(unwrapped_rotation)
            ordered_samples = sorted(self._rotation_samples)
            self._control_rotation = ordered_samples[len(ordered_samples) // 2]
            self._rx_sequence += 1
            self._last_rx_time = monotonic()
        self._rx_ready.set()

    def _get_rcv_snapshot_state(self, wait_until_ready=True, timeout=None):
        self._disarm_sleep_bound_move()
        if wait_until_ready:
            self._wait_for_rx_ready(timeout)
        with self._rx_lock:
            return bytearray(self.rcvPacket), self._rx_sequence

    def _get_rcv_snapshot(self, wait_until_ready=True, timeout=None):
        packet, _rx_sequence = self._get_rcv_snapshot_state(wait_until_ready=wait_until_ready, timeout=timeout)
        return packet

    def _await_manual_response(self, req_packet, timeout):
        _packet, start_rx_sequence = self._get_rcv_snapshot_state(wait_until_ready=False)
        wait_for_new_response = self.isRXThreadRun
        elapsed = 0.0
        while elapsed < timeout:
            packet, rx_sequence = self._get_rcv_snapshot_state(wait_until_ready=False)
            if (not wait_for_new_response or rx_sequence > start_rx_sequence) and packet[4] == len(req_packet) and packet[6] == 2:
                return packet[7]
            sleep(0.01)
            elapsed += 0.01
        return None

    def _get_control_rotation_state(self, wait_until_ready=False, timeout=None):
        if wait_until_ready:
            self._wait_for_rx_ready(timeout)
        with self._rx_lock:
            age = None if self._last_rx_time is None else monotonic() - self._last_rx_time
            return self._control_rotation, self._rx_sequence, age

    def _clear_spin_state(self):
        with self._state_lock:
            self._spin_turn = None
            self._spin_speed = 0
            self._spin_target = None
            self._spin_source_seq = -1

    def _set_spin_state(self, turn, speed, target=None, source_seq=-1):
        with self._state_lock:
            self._spin_turn = turn
            self._spin_speed = speed
            self._spin_target = target
            self._spin_source_seq = source_seq

    def _set_spin_target(self, target, source_seq):
        with self._state_lock:
            self._spin_target = target
            self._spin_source_seq = source_seq

    def _arm_sleep_bound_move(self):
        with self._state_lock:
            self._sleep_bound_move = True

    def _disarm_sleep_bound_move(self):
        with self._state_lock:
            self._sleep_bound_move = False

    def _arm_sleep_bound_turn(self):
        with self._state_lock:
            self._sleep_bound_turn = True

    def _disarm_sleep_bound_turn(self):
        with self._state_lock:
            self._sleep_bound_turn = False

    def _maybe_stop_after_sleep(self):
        with self._state_lock:
            should_stop = self._sleep_bound_move
            self._sleep_bound_move = False
            should_stop_turn = self._sleep_bound_turn
            self._sleep_bound_turn = False
        if self._safe_is_open():
            if should_stop:
                self.move(STOP)
            if should_stop_turn:
                self._stop_turn()

    def _clear_move_with_bend_state(self):
        with self._state_lock:
            self._move_with_bend_vel = 0
            self._move_with_bend_step = 0
            self._move_with_bend_heights = None
            self._move_with_bend_phase = 0
            self._move_with_bend_next_phase_at = 0.0

    def _set_move_with_bend_state(self, move_vel, step, heights):
        with self._state_lock:
            self._move_with_bend_vel = move_vel
            self._move_with_bend_step = step
            self._move_with_bend_heights = list(heights)
            self._move_with_bend_phase = 0
            self._move_with_bend_next_phase_at = 0.0

    def _resolve_move_with_bend_heights(self, bend=None):
        if bend is not None:
            constrained_bend = DefLib.constrain(bend, 20, 90)
            return [constrained_bend] * 4

        with self._state_lock:
            if self._move_with_bend_heights is not None:
                return list(self._move_with_bend_heights)

        pkt = self.makepkt.getCopiedPacket()
        if pkt[15] == 0x02:
            return [
                DefLib.constrain(pkt[16], 20, 90),
                DefLib.constrain(pkt[18], 20, 90),
                DefLib.constrain(pkt[20], 20, 90),
                DefLib.constrain(pkt[22], 20, 90),
            ]
        if pkt[15] == 0x01:
            raw_heights = list(pkt[16:20])
            if any(20 <= value <= 90 for value in raw_heights):
                return [DefLib.constrain(value, 20, 90) for value in raw_heights]
        return [60] * 4

    def _resolve_move_with_bend_args(self, direction_or_vel, vel_or_step=None, bend=None, step=None):
        if direction_or_vel in (FRONT, BACK):
            raise ValueError('move_with_bend() direction form uses FORWARD/BACKWARD or a signed velocity')

        if direction_or_vel in (FORWARD, BACKWARD):
            if vel_or_step is None:
                raise TypeError('move_with_bend() requires speed when direction is used')
            move_vel = self._resolve_move_velocity(direction_or_vel, vel_or_step)
            stride_step = 40 if step is None else step
        else:
            move_vel = self._resolve_move_velocity(direction_or_vel, None)
            if vel_or_step is not None and step is not None:
                raise TypeError('move_with_bend() accepts either positional step or step=, not both')
            stride_step = vel_or_step if vel_or_step is not None else (40 if step is None else step)

        stride_step = DefLib.constrain(abs(stride_step), 0, 90)
        heights = self._resolve_move_with_bend_heights(bend)
        return move_vel, stride_step, heights

    @staticmethod
    def _move_with_bend_interval(move_vel):
        speed = min(max(abs(move_vel), 1), 100)
        return max(0.12, 0.42 - (speed / 100.0) * 0.26)

    @staticmethod
    def _apply_move_with_bend_gait(pkt, move_vel, step, heights, phase):
        if pkt[15] != 0x02:
            pkt[16:24] = bytearray([DefLib.comp(-127)] * 8)
        pkt[15] = 0x02

        stride = step if move_vel >= 0 else -step
        phase_sign = 1 if phase == 0 else -1
        diagonal_a = DefLib.constrain(phase_sign * stride, -90, 90)
        diagonal_b = DefLib.constrain(-phase_sign * stride, -90, 90)
        height_speed = DefLib.constrain(max(abs(move_vel), 20), 0, 100)
        step_speed = DefLib.constrain(max(abs(move_vel), 20), 0, 100)

        for leg_index in range(4):
            pkt[16 + leg_index * 2] = DefLib.constrain(heights[leg_index], 20, 90)
            leg_step = diagonal_a if leg_index in (LEFT_FRONT, RIGHT_BACK) else diagonal_b
            pkt[16 + leg_index * 2 + 1] = DefLib.comp(leg_step)
            pkt[40 + leg_index * 2] = height_speed
            pkt[40 + leg_index * 2 + 1] = step_speed

        pkt[5] = DefLib.checksum(pkt)

    def _resolve_move_with_bend_packet(self, pkt):
        with self._state_lock:
            if self._move_with_bend_vel == 0 or self._move_with_bend_step == 0 or self._move_with_bend_heights is None:
                return False

            move_vel = self._move_with_bend_vel
            stride_step = self._move_with_bend_step
            heights = list(self._move_with_bend_heights)
            now = monotonic()
            interval = self._move_with_bend_interval(move_vel)
            if self._move_with_bend_next_phase_at == 0.0:
                self._move_with_bend_next_phase_at = now + interval
            elif now >= self._move_with_bend_next_phase_at:
                self._move_with_bend_phase = 1 - self._move_with_bend_phase
                self._move_with_bend_next_phase_at = now + interval
            phase = self._move_with_bend_phase

        self._apply_move_with_bend_gait(pkt, move_vel, stride_step, heights, phase)
        return True

    def _snapshot_runtime_state(self):
        with self._state_lock:
            manual_packet = None if self.manualPacket is None else bytearray(self.manualPacket)
            spin_turn = self._spin_turn
            spin_speed = self._spin_speed
            spin_target = self._spin_target
            spin_source_seq = self._spin_source_seq
            led_packet = bytes(self.ledPacket)
        return manual_packet, spin_turn, spin_speed, spin_target, spin_source_seq, led_packet

    def _apply_turn_target(self, pkt, target, degVel=100):
        if pkt[15] != 0x01:
            pkt[16:24] = bytearray([0] * 8)
        pkt[15] = 0x01
        target = max(min(int(target), 32767), -32768)
        pkt[21] = DefLib.constrain(degVel, 0, 100)
        pkt[22] = target & 0xFF
        pkt[23] = (target >> 8) & 0xFF

    def _resolve_spin_target(self, spin_turn, spin_speed, current_target, source_seq):
        control_rotation, rx_seq, age = self._get_control_rotation_state(wait_until_ready=False)
        target = current_target
        if age is not None and age <= self._spin_stale_timeout:
            if target is None or rx_seq != source_seq:
                target = control_rotation + spin_turn
                self._set_spin_target(target, rx_seq)
        elif target is None:
            target = control_rotation + spin_turn
            self._set_spin_target(target, rx_seq)
        return target, spin_speed

    def transmitHandler(self):
        while self.isTXThreadRun:
            manual_packet, spin_turn, spin_speed, spin_target, spin_source_seq, led_packet = self._snapshot_runtime_state()
            if manual_packet is not None:
                if not self._write_serial(manual_packet):
                    break
                sleep(0.1)
                continue

            pkt = self.makepkt.getCopiedPacket()
            if not self._resolve_move_with_bend_packet(pkt) and spin_turn is not None:
                resolved_target, resolved_speed = self._resolve_spin_target(spin_turn, spin_speed, spin_target, spin_source_seq)
                if resolved_target is not None:
                    self._apply_turn_target(pkt, resolved_target, resolved_speed)
                    pkt[5] = DefLib.checksum(pkt)

            if (pkt[14] & 0xC0) == 0xC0:
                if (self.txCnt % 2) == 0:
                    pkt[14] = led_packet[0]
                    pkt[24:40] = led_packet[2:18]
                else:
                    pkt[14] = led_packet[1]
                    pkt[24:40] = led_packet[18:34]
                pkt[5] = DefLib.checksum(pkt)

            if pkt is not None and not self._write_serial(pkt):
                break
            sleep(0.05)
            self.txCnt += 1
        self.isTXThreadRun = False

    def receiveHandler(self):
        while self.isRXThreadRun:
            readData = self._read_serial()
            if readData is None:
                break
            packet = self.parse.packetArrange(readData)
            if not eq(packet, "None"):
                self._update_rx_state(packet)
                if self.receiveCallback is not None:
                    self.receiveCallback(self, packet)
        self.isRXThreadRun = False

    def Open(self, portName="None"):
        if eq(portName, "None"):
            nodes = comports()
            for node in nodes:
                if "USB " in node.description:
                    portName = node.device
                if "Dongle" in node.description:
                    portName = node.device

            if eq(portName, "None"):
                print("Can't find Serial Port")
                return False
        try:
            self.serial = serial.Serial(port=portName, baudrate=115200, timeout=1)
            if self.serial.isOpen():
                self._reset_rx_state()
                with self._state_lock:
                    self.manualPacket = None
                    self._spin_turn = None
                    self._spin_speed = 0
                    self._spin_target = None
                    self._spin_source_seq = -1
                    self._sleep_bound_move = False
                    self._sleep_bound_turn = False
                    self._move_with_bend_vel = 0
                    self._move_with_bend_step = 0
                    self._move_with_bend_heights = None
                    self._move_with_bend_phase = 0
                    self._move_with_bend_next_phase_at = 0.0
                self.txCnt = 0
                self.isTXThreadRun = True
                self.isRXThreadRun = True
                self.txThread = Thread(target=self.transmitHandler, args=(), daemon=False, name=f"RoboDogTX-{self.portIndex}")
                self.rxThread = Thread(target=self.receiveHandler, args=(), daemon=False, name=f"RoboDogRX-{self.portIndex}")
                with self._close_lock:
                    self._close_in_progress = False
                    self._closed = False
                self.txThread.start()
                self.rxThread.start()
                _track_active_dog(self)
                _ensure_sigint_handler()
                _ensure_sleep_hook()
                self._wait_for_rx_ready()
                print("Connected to", portName)
                return True
            print("Can't open " + portName)
            return False
        except BaseException:
            print("Can't open " + portName)
            try:
                if self._safe_is_open():
                    self.serial.close()
            except (AttributeError, OSError, TypeError, serial.SerialException):
                pass
            self.serial = None
            self.isTXThreadRun = False
            self.isRXThreadRun = False
            return False

    def Close(self):
        current = current_thread()
        with self._close_lock:
            if self._close_in_progress:
                return
            if self._closed and self.serial is None and self.txThread is None and self.rxThread is None:
                return
            self._close_in_progress = True
        try:
            self.isTXThreadRun = False
            self.isRXThreadRun = False
            with self._state_lock:
                self.manualPacket = None
                self._spin_turn = None
                self._spin_speed = 0
                self._spin_target = None
                self._spin_source_seq = -1
                self._sleep_bound_move = False
                self._sleep_bound_turn = False
                self._move_with_bend_vel = 0
                self._move_with_bend_step = 0
                self._move_with_bend_heights = None
                self._move_with_bend_phase = 0
                self._move_with_bend_next_phase_at = 0.0

            clear_packet = self.makepkt.clearPacket()
            if self._safe_is_open():
                try:
                    self.serial.write(clear_packet)
                except (AttributeError, OSError, TypeError, serial.SerialException):
                    pass

            if self.serial is not None:
                try:
                    if self.serial.isOpen():
                        self.serial.close()
                except (AttributeError, OSError, TypeError, serial.SerialException):
                    pass

            if self.txThread is not None and self.txThread is not current and self.txThread.is_alive():
                self.txThread.join(timeout=0.5)
            if self.rxThread is not None and self.rxThread is not current and self.rxThread.is_alive():
                self.rxThread.join(timeout=1.5)

            self.serial = None
            self.txThread = None
            self.rxThread = None
            self.txCnt = 0
            self._reset_rx_state()
            _untrack_active_dog(self)
        finally:
            with self._close_lock:
                self._closed = True
                self._close_in_progress = False

    def wait(self, interval=0.1):
        try:
            while self._safe_is_open() and (self.isTXThreadRun or self.isRXThreadRun):
                sleep(interval)
        except KeyboardInterrupt:
            self.Close()
            raise

    def leg(self, what, height, step, height_speed=100, step_speed=100):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        self._clear_spin_state()
        self._clear_move_with_bend_state()
        height = DefLib.constrain(height, 20, 90)
        step = DefLib.constrain(step, -90, 90)
        what = [what] if isinstance(what, int) else what

        def updater(pkt):
            if pkt[15] != 0x02:
                pkt[16:24] = bytearray([DefLib.comp(-127)] * 8)
            pkt[15] = 0x02
            for leg_index in what:
                pkt[16 + leg_index * 2] = height
                pkt[16 + leg_index * 2 + 1] = DefLib.comp(step)
                pkt[40 + leg_index * 2] = DefLib.constrain(height_speed, 0, 100)
                pkt[40 + leg_index * 2 + 1] = DefLib.constrain(step_speed, 0, 100)

        self._update_packet(updater)

    def motor(self, what, shoulder, knee, shld_speed=100, knee_speed=100):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        self._clear_spin_state()
        self._clear_move_with_bend_state()
        shoulder = DefLib.constrain(shoulder, -90, 90)
        knee = DefLib.constrain(knee, -90, 70)
        what = [what] if isinstance(what, int) else what

        def updater(pkt):
            if pkt[15] != 0x03:
                pkt[16:24] = bytearray([DefLib.comp(-127)] * 8)
            pkt[15] = 0x03
            for leg_index in what:
                pkt[16 + leg_index * 2] = DefLib.comp(shoulder)
                pkt[16 + leg_index * 2 + 1] = DefLib.comp(knee)
                pkt[40 + leg_index * 2] = DefLib.constrain(shld_speed, 0, 100)
                pkt[40 + leg_index * 2 + 1] = DefLib.constrain(knee_speed, 0, 100)

        self._update_packet(updater)

    def leg_bend(self, what_or_leftup, bend=None, leftdw=None, rightdw=None):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        self._clear_move_with_bend_state()

        def updater(pkt):
            if pkt[15] != 0x01:
                pkt[16:24] = bytearray([0] * 8)
            pkt[15] = 0x01
            if leftdw is None and rightdw is None and bend is not None:
                what = [what_or_leftup] if isinstance(what_or_leftup, int) else what_or_leftup
                constrained_bend = DefLib.constrain(bend, 20, 90)
                bend_index = (16, 17, 18, 19)
                for leg in what:
                    pkt[bend_index[leg]] = constrained_bend
            else:
                if bend is None or leftdw is None or rightdw is None:
                    raise TypeError('leg_bend() requires either (what, bend) or (leftup, rightup, leftdw, rightdw)')
                pkt[16] = DefLib.constrain(what_or_leftup, 20, 90)
                pkt[17] = DefLib.constrain(leftdw, 20, 90)
                pkt[18] = DefLib.constrain(rightdw, 20, 90)
                pkt[19] = DefLib.constrain(bend, 20, 90)

        self._update_packet(updater)

    def _resolve_move_velocity(self, direction_or_vel, vel=None):
        if vel is None:
            if direction_or_vel == STOP:
                return 0
            return DefLib.constrain(direction_or_vel, -100, 100)

        move_vel = DefLib.constrain(abs(vel), 0, 100)
        if direction_or_vel in (FRONT, FORWARD):
            return move_vel
        if direction_or_vel in (BACK, BACKWARD):
            return -move_vel
        raise ValueError('move direction must be FRONT/FORWARD or BACK/BACKWARD')

    def move(self, direction_or_vel, vel=None):
        self._clear_move_with_bend_state()
        move_vel = self._resolve_move_velocity(direction_or_vel, vel)
        if move_vel == 0:
            self._disarm_sleep_bound_move()
        else:
            self._arm_sleep_bound_move()

        def updater(pkt):
            if pkt[15] != 0x01:
                pkt[16:24] = bytearray([0] * 8)
            pkt[15] = 0x01
            pkt[20] = DefLib.comp(move_vel)

        self._update_packet(updater)

    def move_with_bend(self, direction_or_vel, vel_or_step=None, bend=None, step=None):
        move_vel, stride_step, heights = self._resolve_move_with_bend_args(direction_or_vel, vel_or_step, bend, step)
        if move_vel == 0 or stride_step == 0:
            self.move(STOP)
            return

        self._arm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        self._clear_spin_state()
        self._set_move_with_bend_state(move_vel, stride_step, heights)

        def updater(pkt):
            self._apply_move_with_bend_gait(pkt, move_vel, stride_step, heights, 0)

        self._update_packet(updater)

    def _resolve_turn_request(self, direction_or_degree, degree=None, degVel=100, api_name='rotate'):
        if direction_or_degree in (CW, CCW):
            if degree is None:
                raise TypeError(f'{api_name}() requires degree when direction is CW or CCW')
            turn_or_target = DefLib.constrain(abs(degree), 0, 1000)
            if direction_or_degree == CCW:
                turn_or_target = -turn_or_target
            speed = DefLib.constrain(abs(degVel), 0, 100)
            return turn_or_target, speed

        if degree is None:
            turn_or_target = DefLib.constrain(direction_or_degree, -1000, 1000)
            speed = DefLib.constrain(abs(degVel), 0, 100)
            return turn_or_target, speed

        if degVel != 100:
            raise TypeError(
                f'{api_name}() accepts either (degree, degVel) or (CW/CCW, degree, degVel)'
            )

        turn_or_target = DefLib.constrain(direction_or_degree, -1000, 1000)
        speed = DefLib.constrain(abs(degree), 0, 100)
        return turn_or_target, speed

    def _set_turn_target(self, target, degVel=100):
        def updater(pkt):
            self._apply_turn_target(pkt, target, degVel)

        self._update_packet(updater)

    @staticmethod
    def _is_stop_turn_request(direction_or_degree, degree=None):
        return direction_or_degree == STOP and degree is None

    def _stop_turn(self):
        self._clear_spin_state()
        if self.serial is None or not (self.isTXThreadRun or self.isRXThreadRun):
            return
        control_rotation, _, _ = self._get_control_rotation_state(wait_until_ready=False)
        self._set_turn_target(control_rotation, 0)

    def _wait_for_turn_target(self, target, degVel=100):
        self._set_turn_target(target, degVel)
        if not self.isRXThreadRun or self.serial is None:
            return

        current_rotation, _, _ = self._get_control_rotation_state(wait_until_ready=False)
        distance = abs(target - current_rotation)
        timeout = max(1.0, (distance / max(degVel, 10)) * 1.5)
        elapsed = 0.0
        try:
            while self.isRXThreadRun and elapsed < timeout:
                current_rotation, _, _ = self._get_control_rotation_state(wait_until_ready=False)
                if abs(target - current_rotation) <= 1:
                    break
                sleep(0.01)
                elapsed += 0.01
        except KeyboardInterrupt:
            self.Close()
            raise
        self._stop_turn()

    def _resolve_spin(self, direction_or_turn, speed=None):
        if speed is None:
            speed = abs(direction_or_turn)
            lookahead = DefLib.constrain(direction_or_turn, -180, 180)
        else:
            speed = DefLib.constrain(abs(speed), 0, 100)
            if speed == 0:
                return 0, 0
            if direction_or_turn == CW:
                lookahead = max(30, min(speed, 180))
            elif direction_or_turn == CCW:
                lookahead = -max(30, min(speed, 180))
            else:
                lookahead = DefLib.constrain(direction_or_turn, -180, 180)
        return lookahead, speed

    def spin(self, direction_or_turn, speed=None):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        self._clear_move_with_bend_state()
        if direction_or_turn == STOP:
            self._stop_turn()
            return
        lookahead, speed = self._resolve_spin(direction_or_turn, speed)
        if speed == 0 or lookahead == 0:
            return

        control_rotation, rx_seq, _ = self._get_control_rotation_state(wait_until_ready=True)
        self._set_spin_state(lookahead, speed, control_rotation + lookahead, rx_seq)
        self._set_turn_target(control_rotation + lookahead, speed)

    def rotate(self, direction_or_degree, degree=None, degVel=100):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        turn, degVel = self._resolve_turn_request(direction_or_degree, degree, degVel, api_name='rotate')
        if degVel == 0:
            return
        if turn == 0:
            return

        self._clear_spin_state()
        self._clear_move_with_bend_state()
        start_rotation, _, _ = self._get_control_rotation_state(wait_until_ready=True)
        self._wait_for_turn_target(start_rotation + turn, degVel)

    def turn(self, direction_or_degree, degree=None, degVel=100):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        if self._is_stop_turn_request(direction_or_degree, degree):
            self._stop_turn()
            return
        turn, degVel = self._resolve_turn_request(direction_or_degree, degree, degVel, api_name='turn')
        if degVel == 0:
            return
        if turn == 0:
            self._stop_turn()
            return

        self._clear_spin_state()
        self._clear_move_with_bend_state()
        start_rotation, _, _ = self._get_control_rotation_state(wait_until_ready=True)
        self._set_turn_target(start_rotation + turn, degVel)
        self._arm_sleep_bound_turn()

    def rotate_absolute(self, direction_or_degree, degree=None, degVel=100):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        target, degVel = self._resolve_turn_request(direction_or_degree, degree, degVel, api_name='rotate_absolute')
        if degVel == 0:
            return

        self._clear_spin_state()
        self._clear_move_with_bend_state()
        self._wait_for_turn_target(target, degVel)

    def turn_absolute(self, direction_or_degree, degree=None, degVel=100):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        target, degVel = self._resolve_turn_request(direction_or_degree, degree, degVel, api_name='turn_absolute')
        if degVel == 0:
            return

        self._clear_spin_state()
        self._clear_move_with_bend_state()
        self._set_turn_target(target, degVel)
        self._arm_sleep_bound_turn()

    def rotate_relative(self, direction_or_degree, degree=None, degVel=100):
        return self.rotate(direction_or_degree, degree, degVel)

    def turn_relative(self, direction_or_degree, degree=None, degVel=100):
        return self.turn(direction_or_degree, degree, degVel)

    def gesture(self, action):
        self._disarm_sleep_bound_move()
        self._disarm_sleep_bound_turn()
        self._clear_spin_state()
        self._clear_move_with_bend_state()

        def updater(pkt):
            if pkt[15] != 0x04:
                pkt[16:24] = bytearray([0] * 8)
            pkt[15] = 0x04
            pkt[16] = DefLib.constrain(action, 0, 4)

        self._update_packet(updater)

    def headLEDDraw(self, leftLED, rightLED):
        if not isinstance(leftLED, bytearray) or len(leftLED) != 8:
            return
        if not isinstance(rightLED, bytearray) or len(rightLED) != 8:
            return

        def updater(pkt):
            pkt[14] = (pkt[14] & 0xC0) | 0x81
            pkt[24:32] = rightLED
            pkt[32:40] = leftLED

        with self._state_lock:
            pkt = self._update_packet(updater)
            self.ledPacket[0] = pkt[14]
            self.ledPacket[1] = self.ledPacket[1] | 0x80
            self.ledPacket[2:18] = pkt[24:40]

    def headLEDExp(self, what):
        def updater(pkt):
            pkt[14] = (pkt[14] & 0xC0) | 0x82
            pkt[24] = what

        with self._state_lock:
            pkt = self._update_packet(updater)
            self.ledPacket[0] = pkt[14]
            self.ledPacket[1] = self.ledPacket[1] | 0x80
            self.ledPacket[2:18] = pkt[24:40]

    def headLEDClear(self):
        with self._state_lock:
            body_active = (self.ledPacket[1] & 0x40) == 0x40
            body_led_frame = bytearray(self.ledPacket[18:34])

            def updater(pkt):
                if body_active:
                    pkt[14] = 0x44
                    pkt[24:40] = body_led_frame
                else:
                    pkt[14] = 0
                    pkt[24:40] = bytearray(16)

            self._update_packet(updater)
            self.ledPacket[2:18] = bytearray(16)
            if body_active:
                self.ledPacket[0] = 0x40
                self.ledPacket[1] = 0x44
            else:
                self.ledPacket[0] = 0
                self.ledPacket[1] = 0

    @staticmethod
    def _resolve_body_led_color(red_or_color, green=None, blue=None):
        if green is None and blue is None:
            if isinstance(red_or_color, (tuple, list)) and len(red_or_color) == 3:
                red, green, blue = red_or_color
            else:
                raise TypeError(
                    'bodyLED() requires either (color), (red, green, blue), or legacy (what, red, green, blue)'
                )
        elif green is not None and blue is not None:
            red = red_or_color
        else:
            raise TypeError(
                'bodyLED() requires either (color), (red, green, blue), or legacy (what, red, green, blue)'
            )

        return (
            DefLib.constrain(red, 0, 255),
            DefLib.constrain(green, 0, 255),
            DefLib.constrain(blue, 0, 255),
        )

    def bodyLED(self, red_or_color, green=None, blue=None, legacy_blue=None):
        if legacy_blue is not None:
            red, green, blue = self._resolve_body_led_color(green, blue, legacy_blue)
        else:
            red, green, blue = self._resolve_body_led_color(red_or_color, green, blue)

        def updater(pkt):
            pkt[14] = (pkt[14] & 0xC0) | 0x44
            for n in range(4):
                pkt[24 + n * 4] = red
                pkt[25 + n * 4] = green
                pkt[26 + n * 4] = blue

        with self._state_lock:
            pkt = self._update_packet(updater)
            self.ledPacket[1] = pkt[14]
            self.ledPacket[0] = self.ledPacket[0] | 0x40
            self.ledPacket[18:34] = pkt[24:40]

    def sound(self, thema_or_track, track_or_volume, volume=None):
        thema, track, volume = _resolve_sound_components(thema_or_track, track_or_volume, volume)

        def updater(pkt):
            trackID = 0 if (pkt[7] & 0x80) == 0x80 else 0x80
            pkt[7] = (thema * 10 + track) | trackID
            pkt[8] = volume

        self._update_packet(updater)

    def ledPrint(self, leftChar, rightChar):
        def updater(pkt):
            pkt[14] = 3
            pkt[32] = leftChar
            pkt[24] = rightChar

        self._update_packet(updater)

    def txManualPacket(self, pkt, rxHead):
        if pkt is not None:
            pkt = bytearray(pkt)
            pkt[5] = DefLib.checksum(pkt)
        self.parse.setManualHead(rxHead)
        with self._state_lock:
            self.manualPacket = pkt

    def ai_mode(self, mode=0, timeout=10.0):
        req_packet = bytearray(0x20)
        req_packet[:7] = [0x26, 0xA8, 0x14, 0x87, 0x20, 0x00, 0x01]
        req_packet[7] = 0x10 | mode
        req_packet[5] = DefLib.checksum(req_packet)
        with self._state_lock:
            self.manualPacket = req_packet
        try:
            ret_mode = self._await_manual_response(req_packet, timeout)
            if ret_mode is not None:
                with self._state_lock:
                    self.manualPacket = None
                return ret_mode
        except KeyboardInterrupt:
            with self._state_lock:
                self.manualPacket = None
            self.Close()
            raise
        with self._state_lock:
            self.manualPacket = None
        return None

    def face_tracking(self, class_type=1, timeout=10.0):
        req_packet = bytearray(0x20)
        req_packet[:7] = [0x26, 0xA8, 0x14, 0x87, 0x20, 0x00, 0x01]
        req_packet[7] = 0x12
        req_packet[8] = 0x30 | class_type
        req_packet[5] = DefLib.checksum(req_packet)
        with self._state_lock:
            self.manualPacket = req_packet
        try:
            ret_mode = self._await_manual_response(req_packet, timeout)
            if ret_mode is not None:
                with self._state_lock:
                    self.manualPacket = None
                return ret_mode
        except KeyboardInterrupt:
            with self._state_lock:
                self.manualPacket = None
            self.Close()
            raise
        with self._state_lock:
            self.manualPacket = None
        return None

    def get_battery(self):
        return self._get_rcv_snapshot()[6]

    def get_distance(self):
        return self._get_rcv_snapshot()[7]

    def get_tilt(self):
        packet = self._get_rcv_snapshot()
        return [DefLib.toSigned8(packet[8]), DefLib.toSigned8(packet[9])]

    def get_rotation(self):
        packet = self._get_rcv_snapshot()
        return DefLib.toSigned16((packet[11] << 8) | packet[10])

    def is_button_pressed(self):
        return self._get_rcv_snapshot()[12]&0x01 == 1

    def is_ai_alive(self):
        return self._get_rcv_snapshot()[13] == 1

    def get_face_recognition_class(self):
        packet = self._get_rcv_snapshot()
        if packet[14] != 0x01:
            return 0
        return packet[15]

    def is_face_tracking(self):
        packet = self._get_rcv_snapshot()
        if packet[14] != 0x02:
            return False
        return packet[15] == 1

    def get_color_class(self):
        packet = self._get_rcv_snapshot()
        if packet[14] != 0x03:
            return 0
        return packet[15]

    def get_qrcode(self):
        packet = self._get_rcv_snapshot()
        if packet[14] != 0x04:
            return None
        if packet[15] != 0x01:
            return None
        data = bytes(packet[16:24])
        end = data.find(0)
        if end == -1:
            end = len(data)
        return data[:end].decode("utf-8")

    def get_object_position(self):
        packet = self._get_rcv_snapshot()
        if packet[14] == 0x00:
            return [0, 0]
        if packet[14] == 0x04:
            return [0, 0]
        return [DefLib.toSigned8(packet[16]), DefLib.toSigned8(packet[17])]

    def get_ai_data(self):
        return self._get_rcv_snapshot()[14:24]


def getWindowsPortList():
    nodes = comports()
    portList = []
    for node in nodes:
        if "USB " in node.description:
            portList.append(node.device)
        if "Dongle" in node.description:
            portList.append(node.device)

    if len(portList) == 0:
        print("Can't find Serial Port")
        return None
    return portList


def getLinuxPortList():
    try:
        output = subprocess.check_output('ls /dev/ttyACM*', shell=True)
        return tuple(output.decode().strip().split())
    except subprocess.CalledProcessError:
        return ()


def RoboDogMultiOpen(nameList=None, receiveCallback=None):
    if nameList is None:
        if platform.system() == 'Linux':
            nameList = getLinuxPortList()
        else:
            nameList = getWindowsPortList()
    if nameList is None:
        return None
    howMany = len(nameList)
    if howMany == 0:
        return None
    howMany = 10 if howMany > 10 else howMany
    robodog = []
    for n in range(howMany):
        robodog.append(RoboDog(receiveCallback, n))
        if not robodog[n].Open(nameList[n]):
            robodog.pop()
            RoboDogMultiClose(robodog)
            exit()
    return robodog



def RoboDogMultiClose(robodog):
    if robodog is None:
        return
    howMany = len(robodog)
    for n in range(len(robodog)):
        robodog[n].gesture(0)
    sleep(0.5)
    for n in range(howMany):
        robodog[n].Close()



def _resolve_multi_order_indices(dogs, order=ORDER_LOW_TO_HIGH):
    if dogs is None:
        return []

    if order in (None, ORDER_LOW_TO_HIGH):
        return range(len(dogs))
    if order == ORDER_HIGH_TO_LOW:
        return range(len(dogs) - 1, -1, -1)
    raise ValueError(
        "order must be ORDER_LOW_TO_HIGH or ORDER_HIGH_TO_LOW"
    )



def _resolve_multi_interval(interval):
    try:
        delay = float(interval)
    except (TypeError, ValueError):
        raise TypeError("interval must be a number")
    return 0.0 if delay <= 0 else delay



def _dispatch_multi(dogs, callback, interval=0.0, order=ORDER_LOW_TO_HIGH):
    indices = list(_resolve_multi_order_indices(dogs, order))
    delay = _resolve_multi_interval(interval)
    last_index = len(indices) - 1
    for position, dog_index in enumerate(indices):
        callback(dogs[dog_index])
        if delay > 0.0 and position < last_index:
            _ORIGINAL_TIME_SLEEP(delay)



def _dispatch_multi_parallel(dogs, callback):
    if dogs is None:
        return

    start_event = Event()
    workers = []
    errors = Queue()

    def worker(dog):
        start_event.wait()
        try:
            callback(dog)
        except BaseException as exc:
            errors.put(exc)

    for dog in dogs:
        thread = Thread(target=worker, args=(dog,), daemon=False)
        workers.append(thread)
        thread.start()

    start_event.set()

    for thread in workers:
        thread.join()

    if not errors.empty():
        raise errors.get()



def _invoke_multi_body_led(dog, red_or_color, green=None, blue=None, legacy_blue=None):
    if legacy_blue is not None:
        dog.bodyLED(red_or_color, green, blue, legacy_blue)
        return
    if green is None and blue is None:
        dog.bodyLED(red_or_color)
        return
    dog.bodyLED(red_or_color, green, blue)



def _invoke_multi_move(dog, direction_or_vel, vel=None):
    if vel is None:
        dog.move(direction_or_vel)
        return
    dog.move(direction_or_vel, vel)



def _invoke_multi_spin(dog, direction_or_turn, speed=None):
    if speed is None:
        dog.spin(direction_or_turn)
        return
    dog.spin(direction_or_turn, speed)



def _invoke_multi_rotate(method, direction_or_degree, degree=None, degVel=100):
    if degree is None:
        method(direction_or_degree)
        return
    if degVel == 100:
        method(direction_or_degree, degree)
        return
    method(direction_or_degree, degree, degVel)



def _invoke_multi_turn(method, direction_or_degree, degree=None, degVel=100):
    _invoke_multi_rotate(method, direction_or_degree, degree, degVel)



def multi_leg_bend(dogs, what_or_leftup, bend=None, leftdw=None, rightdw=None):
    for n in range(len(dogs)):
        dogs[n].leg_bend(what_or_leftup, bend, leftdw, rightdw)



def multi_move(dogs, direction_or_vel, vel=None):
    for n in range(len(dogs)):
        _invoke_multi_move(dogs[n], direction_or_vel, vel)



def multi_move_sequential(dogs, interval, order, direction_or_vel, vel=None):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_move(dog, direction_or_vel, vel),
        interval,
        order,
    )



def multi_move_with_bend(dogs, direction_or_vel, vel_or_step=None, bend=None, step=None):
    for n in range(len(dogs)):
        dogs[n].move_with_bend(direction_or_vel, vel_or_step, bend, step)



def multi_spin(dogs, direction_or_turn, speed=None):
    for n in range(len(dogs)):
        _invoke_multi_spin(dogs[n], direction_or_turn, speed)



def multi_spin_sequential(dogs, interval, order, direction_or_turn, speed=None):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_spin(dog, direction_or_turn, speed),
        interval,
        order,
    )



def multi_rotate(dogs, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi_parallel(
        dogs,
        lambda dog: _invoke_multi_rotate(dog.rotate, direction_or_degree, degree, degVel),
    )


def multi_rotate_sequential(dogs, interval, order, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_rotate(dog.rotate, direction_or_degree, degree, degVel),
        interval,
        order,
    )



def multi_rotate_absolute(dogs, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi_parallel(
        dogs,
        lambda dog: _invoke_multi_rotate(dog.rotate_absolute, direction_or_degree, degree, degVel),
    )


def multi_rotate_absolute_sequential(dogs, interval, order, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_rotate(dog.rotate_absolute, direction_or_degree, degree, degVel),
        interval,
        order,
    )



def multi_rotate_relative(dogs, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi_parallel(
        dogs,
        lambda dog: _invoke_multi_rotate(dog.rotate_relative, direction_or_degree, degree, degVel),
    )



def multi_rotate_relative_sequential(dogs, interval, order, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_rotate(dog.rotate_relative, direction_or_degree, degree, degVel),
        interval,
        order,
    )



def multi_turn(dogs, direction_or_degree, degree=None, degVel=100):
    for n in range(len(dogs)):
        _invoke_multi_turn(dogs[n].turn, direction_or_degree, degree, degVel)



def multi_turn_sequential(dogs, interval, order, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_turn(dog.turn, direction_or_degree, degree, degVel),
        interval,
        order,
    )



def multi_turn_absolute(dogs, direction_or_degree, degree=None, degVel=100):
    for n in range(len(dogs)):
        _invoke_multi_turn(dogs[n].turn_absolute, direction_or_degree, degree, degVel)



def multi_turn_absolute_sequential(dogs, interval, order, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_turn(dog.turn_absolute, direction_or_degree, degree, degVel),
        interval,
        order,
    )



def multi_turn_relative(dogs, direction_or_degree, degree=None, degVel=100):
    for n in range(len(dogs)):
        _invoke_multi_turn(dogs[n].turn_relative, direction_or_degree, degree, degVel)



def multi_turn_relative_sequential(dogs, interval, order, direction_or_degree, degree=None, degVel=100):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_turn(dog.turn_relative, direction_or_degree, degree, degVel),
        interval,
        order,
    )



def multi_gesture(dogs, action):
    for n in range(len(dogs)):
        dogs[n].gesture(action)



def multi_gesture_sequential(dogs, interval, order, action):
    _dispatch_multi(
        dogs,
        lambda dog: dog.gesture(action),
        interval,
        order,
    )



def multi_headLEDDraw(dogs, leftLED, rightLED):
    for n in range(len(dogs)):
        dogs[n].headLEDDraw(leftLED, rightLED)



def multi_headLEDDraw_sequential(dogs, interval, order, leftLED, rightLED):
    _dispatch_multi(
        dogs,
        lambda dog: dog.headLEDDraw(leftLED, rightLED),
        interval,
        order,
    )



def multi_headLEDExp(dogs, what):
    for n in range(len(dogs)):
        dogs[n].headLEDExp(what)



def multi_headLEDExp_sequential(dogs, interval, order, what):
    _dispatch_multi(
        dogs,
        lambda dog: dog.headLEDExp(what),
        interval,
        order,
    )



def multi_bodyLED(dogs, red_or_color, green=None, blue=None, legacy_blue=None):
    for n in range(len(dogs)):
        _invoke_multi_body_led(dogs[n], red_or_color, green, blue, legacy_blue)



def multi_bodyLED_sequential(dogs, interval, order, red_or_color, green=None, blue=None, legacy_blue=None):
    _dispatch_multi(
        dogs,
        lambda dog: _invoke_multi_body_led(dog, red_or_color, green, blue, legacy_blue),
        interval,
        order,
    )



def multi_sound(dogs, thema_or_track, track_or_volume, volume=None):
    for n in range(len(dogs)):
        dogs[n].sound(thema_or_track, track_or_volume, volume)


def multi_ledPrint(dogs, leftChar, rightChar):
    for n in range(len(dogs)):
        dogs[n].ledPrint(leftChar, rightChar)



def multi_ledPrint_sequential(dogs, interval, order, leftChar, rightChar):
    _dispatch_multi(
        dogs,
        lambda dog: dog.ledPrint(leftChar, rightChar),
        interval,
        order,
    )



def multi_leg(dogs, what, height, step, height_speed=100, step_speed=100):
    for n in range(len(dogs)):
        dogs[n].leg(what, height, step, height_speed, step_speed)



def multi_motor(dogs, what, shoulder, knee, shld_speed=100, knee_speed=100):
    for n in range(len(dogs)):
        dogs[n].motor(what, shoulder, knee, shld_speed, knee_speed)
