import struct, sys, time, json, io, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from main import decode_telemetry, decode_fault, DiagReassembler, GraderNDJSONAdapter

def ff(total, first6):
    return bytes([0x10 | ((total >> 8) & 0xF), total & 0xFF]) + bytes(first6)
def cf(seq, chunk7):
    c = bytes(chunk7)
    return bytes([0x20 | (seq & 0xF)]) + c + bytes(7 - len(c))

# TP-02
d = decode_telemetry(bytes.fromhex("E80F000000000000"))
assert d["voltage"] == 407.2, d
# escala completa
p = struct.pack("<HHBBH", 4123, 11855, 87, 0b101, 4821)
d = decode_telemetry(p)
assert (d["voltage"], d["current"], d["temp_c"], d["seq"]) == (412.3, 118.55, 47, 4821), d
assert (d["enabled"], d["fault"], d["derated"]) == (True, False, True), d
assert decode_telemetry(b"short") is None
# fault
assert decode_fault(bytes([2, 1, 0, 0, 0, 0, 0, 0])) == (2, 1)
assert decode_fault(bytes([9, 1, 0, 0, 0, 0, 0, 0])) is None
assert decode_fault(bytes([1, 9, 0, 0, 0, 0, 0, 0])) is None
# multitrama feliz (concurrencia: 2 modulos)
msg = b"SN:PMU-4471-A FW:2.3.1"  # 21 B
for cid in (0x6F0, 0x6F1):
    r = DiagReassembler()
    now = time.monotonic()
    assert r.feed(cid, ff(len(msg), msg[:6]), now, 0) is None
    out = None
    seq, off = 1, 6
    while off < len(msg):
        out = r.feed(cid, cf(seq, msg[off:off+7]), now, 0)
        off += 7; seq = (seq + 1) % 16
        if seq == 0 and off < len(msg): pass
    assert out == msg.decode(), (cid, out)
# TP-03 huerfana
r = DiagReassembler()
assert r.feed(0x6F0, cf(1, b"ABCDEFG"), time.monotonic(), 0) is None
# TP-04 oversized
r = DiagReassembler()
assert r.feed(0x6F0, ff(100, b"123456"), time.monotonic(), 0) is None
# out-of-order
r = DiagReassembler(); now = time.monotonic()
r.feed(0x6F0, ff(21, msg[:6]), now, 0)
assert r.feed(0x6F0, cf(2, msg[6:13]), now, 0) is None  # esperaba 1
assert r.feed(0x6F0, cf(1, msg[6:13]), now, 0) is None  # IDLE -> huerfana
# restart
r = DiagReassembler(); now = time.monotonic()
msg2 = b"HELLO-WORLD-12345"  # 17 B
r.feed(0x6F2, ff(21, msg[:6]), now, 0)
assert r.feed(0x6F2, ff(len(msg2), msg2[:6]), now, 0) is None
o = r.feed(0x6F2, cf(1, msg2[6:13]), now, 0)
o = r.feed(0x6F2, cf(2, msg2[13:]), now, 0)
assert o == msg2.decode(), o
# TTL (TP-05)
r = DiagReassembler(ttl=0.05)
r.feed(0x6F3, ff(21, msg[:6]), time.monotonic(), 0)
time.sleep(0.08)
r.check_timeouts(time.monotonic())
assert r.feed(0x6F3, cf(1, msg[6:13]), time.monotonic(), 0) is None
# seq wrap 15->0->1
r = DiagReassembler()
big = bytes(range(60))
now = time.monotonic()
r.feed(0x6F0, ff(60, big[:6]), now, 0)
seq, off, done = 1, 6, None
while off < 60:
    done = r.feed(0x6F0, cf(seq, big[off:off+7]), now, 0)
    off += 7; seq = (seq + 1) % 16
assert done is not None and len(done) == 60
# adapter no contamina stdout
import contextlib
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    GraderNDJSONAdapter().emit_telemetry(0, {"seq": 1, "voltage": 1.0, "current": 1.0, "temp_c": 0, "enabled": True, "fault": False, "derated": False})
line = buf.getvalue().strip()
assert json.loads(line)["type"] == "telemetry", line
print("ALL TESTS OK")
