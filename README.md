# DeepSea EV Charger — CAN Diagnostic Tool

Monitor liviano en tiempo real para cargadores rápidos DC Nivel 3. Solo recepción. Sin dependencias externas (stdlib Python 3).

## Ejecución (en Raspberry Pi 5)

```bash
# vcan0 activa desde el inicio; si no:
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

python3 main.py --iface vcan0            # dashboard interactivo
python3 main.py --iface vcan0 --grader   # modo evaluador NDJSON
```

## Protocolo implementado

- `0x100–0x103` telemetría 8B LE: `voltage=raw*0.1` (`round 1`), `current=raw*0.01` (`round 2`), `temp=raw-40`, `status bit0 enabled/bit1 fault/bit2 derated`, `seq u16`.
- `0x1F0` falla 8B: `byte0 module 0-3`, `byte1 code 1-4` (1 overtemp, 2 overvoltage, 3 undervoltage, 4 isolation). Otros valores se ignoran.
- `0x6F0–0x6F3` multitrama tipo ISO 15765-2: FF `0x10|(len>>8), len&0xFF + 6B`, CF `0x20|seq + 7B`, `seq 1..15,0,1...`, `len 8..64`.
- Ruido `0x200–0x2FF` y cualquier ID fuera de `{0x100-0x103,0x1F0,0x6F0-0x6F3}` se ignora. DLC `!=8` se ignora. Nunca se transmite.

## Arquitectura (4 capas)

1. **Network I/O**: `socket(PF_CAN, SOCK_RAW, CAN_RAW)`, `bind(iface)`, `select` 200 ms, `struct <IB3x8s`.
2. **Filter & Dispatch**: máscara a 11-bit, descarte temprano de ruido, conteo `frames_processed` por cada trama recibida.
3. **Decoding & FSM**: `decode_telemetry` / `decode_fault` + `DiagReassembler` con 1 FSM IDLE/BUILDING por cada `0x6F0-0x6F3`.
4. **Presentation**: patrón Adapter — `GraderNDJSONAdapter` (stdout NDJSON + flush) vs dashboard ANSI (stdout normal). Logs siempre a stderr.

## FSM multitrama

- IDLE + CF → ignora (huérfana).
- IDLE + FF `len>64` o `len<=7` → rechaza.
- IDLE + FF válida → guarda 6B, `expected=1`, `deadline=now+1.0s` → BUILDING.
- BUILDING + FF válida → reinicia con la nueva (restart); si es oversized → IDLE.
- BUILDING + CF `seq!=expected` → descarta → IDLE.
- BUILDING + timeout TTL → libera → IDLE (chequeo cada ciclo + lazy al recibir).
- BUILDING + CF correcta → acumula; si `len(buf)>=total` emite `string` UTF-8 y → IDLE, sino `expected=(expected+1)%16`.
- Memoria O(1): 4 buffers ≤64 B + TTL. Sin emisión en abandonos.

Decisión de diseño a defender: TTL por polling en el mismo hilo (sin hilos/timers) para mantener CPU ligera y evitar condiciones de carrera; `select` con timeout sirve a la vez para stats periódicos, expiración y refresco de dashboard.

## Modo `--grader` (contrato ADAPTER.md)

- `argparse --grader` desactiva dashboard/ANSI.
- stdout solo NDJSON, 1 JSON por línea + `flush`. Todo log a stderr.
- `telemetry` por cada `0x100-0x103` válida; `fault` por cada `0x1F0` válida; `diag_complete {can_id:"0x6f0", string, ts_ns:monotonic_ns()}` al completar; `stats {frames_processed}` cada 2 s + una final en `finally` (SIGINT/SIGTERM incluidos).

## Verificación local (sin vcan)

```bash
python3 tests/test_decode.py
```

Monitoreo en Pi: `candump vcan0`, `cansend vcan0 100#E80F000000000000`.
