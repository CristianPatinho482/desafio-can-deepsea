#!/usr/bin/env python3
"""DeepSea EV Charger - CAN Diagnostic Tool.
Stdlib only: socket, struct, select, argparse, json, time, sys, signal.
Receive-only. Never transmits on the bus.

  Normal : python3 main.py --iface vcan0
  Grader : python3 main.py --iface vcan0 --grader
"""
import argparse
import json
import select
import signal
import socket
import struct
import sys
import time
from abc import ABC, abstractmethod

CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = 16

TELEMETRY_IDS = {0x100, 0x101, 0x102, 0x103}
FAULT_ID = 0x1F0
DIAG_IDS = {0x6F0, 0x6F1, 0x6F2, 0x6F3}
VALID_IDS = TELEMETRY_IDS | {FAULT_ID} | DIAG_IDS

TTL_SEC = 1.0
STATS_PERIOD_SEC = 2.0
MAX_DIAG_LEN = 64

# ---------------------------------------------------------------- adapters

class OutputAdapter(ABC):
    @abstractmethod
    def emit_telemetry(self, module_id: int, data: dict): ...
    @abstractmethod
    def emit_fault(self, module_id: int, code: int): ...
    @abstractmethod
    def emit_diag_complete(self, can_id: str, message: str, ts_ns: int): ...
    @abstractmethod
    def emit_stats(self, frames_processed: int): ...


class GraderNDJSONAdapter(OutputAdapter):
    """NDJSON a stdout, una linea por evento, flush inmediato."""

    def _emit(self, obj: dict):
        sys.stdout.write(json.dumps(obj) + "\n")
        sys.stdout.flush()

    def emit_telemetry(self, module_id: int, data: dict):
        self._emit({"type": "telemetry", "module": module_id, **data})

    def emit_fault(self, module_id: int, code: int):
        self._emit({"type": "fault", "module": module_id, "code": code})

    def emit_diag_complete(self, can_id: str, message: str, ts_ns: int):
        self._emit({"type": "diag_complete", "can_id": can_id,
                    "string": message, "ts_ns": ts_ns})

    def emit_stats(self, frames_processed: int):
        self._emit({"type": "stats", "frames_processed": frames_processed})


def log_stderr(msg: str):
    sys.stderr.write(msg + "\n")
    sys.stderr.flush()

# ---------------------------------------------------------------- decoders

def decode_telemetry(payload: bytes) -> dict | None:
    """Payload de 8 bytes -> dict listo para evento telemetry. None si invalido."""
    if len(payload) != 8:
        return None
    try:
        voltage_raw, current_raw, temp_raw, status, seq = struct.unpack("<HHBBH", payload[:8])
    except struct.error:
        return None
    voltage = round(voltage_raw * 0.1, 1)
    current = round(current_raw * 0.01, 2)
    return {
        "seq": int(seq),
        "voltage": float(voltage),
        "current": float(current),
        "temp_c": int(temp_raw) - 40,
        "enabled": bool(status & 0x01),
        "fault": bool(status & 0x02),
        "derated": bool(status & 0x04),
    }


def decode_fault(payload: bytes) -> tuple | None:
    """Payload 8 bytes ID 0x1F0 -> (module, code). None si invalido."""
    if len(payload) != 8:
        return None
    module_id, code = payload[0], payload[1]
    if not (0 <= module_id <= 3):
        return None
    if code not in (1, 2, 3, 4):
        return None
    return (int(module_id), int(code))

# ---------------------------------------------------------------- FSM ISO-15765-2 simplificado

class _ReasmState:
    __slots__ = ("active", "total_len", "buf", "expected_seq", "deadline")

    def __init__(self):
        self.active = False      # False=IDLE, True=BUILDING
        self.total_len = 0
        self.buf = bytearray()
        self.expected_seq = 1
        self.deadline = 0.0


class DiagReassembler:
    """Un FSM IDLE/BUILDING por cada CAN-ID 0x6F0-0x6F3. Memoria O(1), TTL 1.0 s."""

    def __init__(self, ttl: float = TTL_SEC):
        self.ttl = ttl
        self._st = {cid: _ReasmState() for cid in DIAG_IDS}

    def _is_first_frame(self, b0: int) -> bool:
        return (b0 >> 4) == 0x1

    def _is_consec_frame(self, b0: int) -> bool:
        return (b0 >> 4) == 0x2

    def check_timeouts(self, now: float):
        for st in self._st.values():
            if st.active and now >= st.deadline:
                st.active = False
                st.buf.clear()
                st.total_len = 0

    def feed(self, can_id: int, data: bytes, now: float, now_ns: int) -> str | None:
        """Procesa una trama de 8 bytes de un ID 0x6F0-0x6F3.
        Retorna el string reensamblado al completarse, o None.
        Nunca lanza: anomalias se ignoran silenciosamente."""
        try:
            if can_id not in self._st or len(data) != 8:
                return None
            st = self._st[can_id]
            # Expiracion perezosa antes de procesar
            if st.active and now >= st.deadline:
                st.active = False
                st.buf.clear()
                st.total_len = 0
            b0 = data[0]
            if self._is_first_frame(b0):
                total = ((b0 & 0x0F) << 8) | data[1]
                # Anomalia 3: longitud excesiva / longitud no valida
                if total > MAX_DIAG_LEN or total <= 7:
                    st.active = False
                    st.buf.clear()
                    st.total_len = 0
                    return None
                # Anomalia 2: reinicio — descarta intento previo
                st.active = True
                st.total_len = total
                st.buf = bytearray(data[2:8])  # primeros 6 bytes
                st.expected_seq = 1
                st.deadline = now + self.ttl
                if len(st.buf) >= st.total_len:
                    msg = bytes(st.buf[:st.total_len]).decode("utf-8", errors="replace")
                    st.active = False
                    st.buf.clear()
                    return msg
                return None
            if self._is_consec_frame(b0):
                seq = b0 & 0x0F
                if not st.active:
                    return None  # Anomalia 1: huerfana
                if seq != st.expected_seq:
                    # Anomalia 4: fuera de orden -> abandonar
                    st.active = False
                    st.buf.clear()
                    st.total_len = 0
                    return None
                need = st.total_len - len(st.buf)
                take = 7 if need >= 7 else need
                st.buf.extend(data[1:1 + take])
                if len(st.buf) >= st.total_len:
                    msg = bytes(st.buf[:st.total_len]).decode("utf-8", errors="replace")
                    st.active = False
                    st.buf.clear()
                    st.total_len = 0
                    return msg
                st.expected_seq = (st.expected_seq + 1) % 16
                return None
            return None  # nibble desconocido: ignorar
        except Exception as e:
            log_stderr(f"[reassembler] ignored error: {e}")
            return None

# ---------------------------------------------------------------- CAN I/O

def open_can_socket(iface: str):
    try:
        s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    except (AttributeError, OSError) as e:
        raise RuntimeError(f"SocketCAN no disponible ({e})") from e
    try:
        s.bind((iface,))
    except OSError as e:
        raise RuntimeError(f"No se pudo bindear {iface}: {e}") from e
    s.setblocking(False)
    return s


def recv_frame(sock) -> tuple | None:
    """Lee una trama. Retorna (can_id, dlc, data8) o None si no hay datos."""
    try:
        raw, _ = sock.recvfrom(CAN_FRAME_SIZE)
    except BlockingIOError:
        return None
    except OSError as e:
        log_stderr(f"[can] recv error: {e}")
        return None
    if len(raw) < CAN_FRAME_SIZE:
        return None
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw[:CAN_FRAME_SIZE])
    # Quita flags EFF/RTR/ERR; IDs del desafio son estandar 11-bit
    if can_id & 0x80000000:
        can_id &= 0x1FFFFFFF
    else:
        can_id &= 0x7FF
    dlc = min(int(dlc), 8)
    return (int(can_id), int(dlc), bytes(data[:8]))

# ---------------------------------------------------------------- dashboard

class DashboardState:
    def __init__(self):
        self.telemetry = {}   # module -> dict
        self.faults = {}      # module -> code
        self.diag = {}        # can_id -> string
        self.frames = 0

    def render(self):
        lines = []
        lines.append("\x1b[2J\x1b[H",)
        lines.append("DeepSea EV Charger - CAN Diagnostic  (solo recepcion)")
        lines.append(f"frames_processed: {self.frames}")
        lines.append("-" * 60)
        lines.append("MOD  SEQ    VOLT[V]  CURR[A]  TEMP  EN FLT DER")
        for m in range(4):
            t = self.telemetry.get(m)
            if t is None:
                lines.append(f"{m:<4} --     --       --       --    -- --   --")
            else:
                lines.append(
                    f"{m:<4} {t['seq']:<6} {t['voltage']:<8} {t['current']:<8}"
                    f" {t['temp_c']:<5} {int(t['enabled'])}  {int(t['fault'])}   {int(t['derated'])}"
                )
        lines.append("-" * 60)
        lines.append(f"faults: {self.faults if self.faults else '{}'}")
        for cid in sorted(DIAG_IDS):
            s = self.diag.get(cid, "")
            lines.append(f"0x{cid:03x}: {s[:64]}")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

# ---------------------------------------------------------------- main loop

def parse_args(argv=None):
    p = argparse.ArgumentParser(description="DeepSea CAN Diagnostic Tool")
    p.add_argument("--iface", default="vcan0", help="interfaz SocketCAN")
    p.add_argument("--grader", action="store_true", help="modo evaluador NDJSON")
    return p.parse_args(argv)


def run(iface: str, grader: bool):
    adapter: OutputAdapter | None = GraderNDJSONAdapter() if grader else None
    dash = DashboardState() if not grader else None
    reassembler = DiagReassembler()
    sock = open_can_socket(iface)
    log_stderr(f"[init] iface={iface} grader={grader}")

    stop = False

    def _on_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    frames_processed = 0
    last_stats = time.monotonic()
    last_draw = 0.0

    try:
        while not stop:
            r, _, _ = select.select([sock], [], [], 0.2)
            now = time.monotonic()
            if r:
                # Drena todo lo disponible sin bloquear
                while True:
                    fr = recv_frame(sock)
                    if fr is None:
                        break
                    can_id, dlc, data = fr
                    frames_processed += 1
                    try:
                        if can_id in TELEMETRY_IDS and dlc == 8:
                            module = can_id - 0x100
                            dec = decode_telemetry(data)
                            if dec is not None:
                                if grader:
                                    adapter.emit_telemetry(module, dec)
                                else:
                                    dash.telemetry[module] = dec
                                    dash.frames = frames_processed
                        elif can_id == FAULT_ID and dlc == 8:
                            f = decode_fault(data)
                            if f is not None:
                                module, code = f
                                if grader:
                                    adapter.emit_fault(module, code)
                                else:
                                    dash.faults[module] = code
                                    dash.frames = frames_processed
                        elif can_id in DIAG_IDS and dlc == 8:
                            s = reassembler.feed(can_id, data, now, time.monotonic_ns())
                            if s is not None:
                                ts_ns = time.monotonic_ns()
                                cid_str = f"0x{can_id:03x}"
                                if grader:
                                    adapter.emit_diag_complete(cid_str, s, ts_ns)
                                else:
                                    dash.diag[can_id] = s
                                    dash.frames = frames_processed
                        else:
                            # Ruido 0x200-0x2FF u otros: ignorar (TP-01)
                            if not grader and dash is not None:
                                dash.frames = frames_processed
                    except Exception as e:
                        # RNF-03: ninguna anomalia colapsa el proceso
                        log_stderr(f"[loop] frame handling error: {e}")
            # TTL
            reassembler.check_timeouts(time.monotonic())
            # stats periodico cada ~2 s
            now2 = time.monotonic()
            if now2 - last_stats >= STATS_PERIOD_SEC:
                if grader:
                    adapter.emit_stats(frames_processed)
                else:
                    dash.frames = frames_processed
                last_stats = now2
            # dashboard
            if not grader and now2 - last_draw >= 0.5:
                dash.frames = frames_processed
                try:
                    dash.render()
                except Exception as e:
                    log_stderr(f"[dash] render error: {e}")
                last_draw = now2
    finally:
        # stats final obligatorio (exactamente una vez mas al cerrar)
        try:
            if grader and adapter is not None:
                adapter.emit_stats(frames_processed)
        except Exception as e:
            log_stderr(f"[shutdown] stats error: {e}")
        try:
            sock.close()
        except Exception:
            pass
        log_stderr(f"[exit] frames_processed={frames_processed}")


def main(argv=None):
    args = parse_args(argv)
    run(args.iface, args.grader)


if __name__ == "__main__":
    main()
