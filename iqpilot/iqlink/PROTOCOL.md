# iqlink ↔ IQ-link (BLE-only)

> **English is authoritative.** Chinese section below is a summary.  
> Field mapping lives in `protocol.py`. This file is the product contract (SSOT).

## Product model

openpilot / IQ.Pilot has **no navigation session**. The car does not own “start/cancel navigation”.

| Side | Role |
|------|------|
| Amap Auto (amapauto) | Whether the user is navigating lives only in that APK |
| IQ-link (phone) | Pushes **live parameters** (road limit, lights, TBT distance, …) over BLE; does **not** clear parameters when navigation stops |
| Device | Applies parameters only when content **changes**; otherwise keeps the last execution snapshot. If none → follow lead / model |

Notes:

- Do not model the device as nav on/off.
- There is **no** “cancel navigation” product control on the car.
- Legacy param names such as `NavigationActive` / `IqlinkExclusive` are implementation leftovers — not product features.
- **Connected (product / QA):** phone ↔ device can communicate = device HMAC `LinkState=2`. Phone GATT “connected” alone is not enough.

## Transport

**Live parameters use BLE GATT only.** Device Wi‑Fi STA (internet / SSH) is unrelated to Partner push.

Deprecated (do not use as fallback): UDP 7705 / 7706, TCP 7713.

## Longitudinal behavior (summary)

- **Change-driven:** identical `data` payloads do not refresh the execution snapshot (link heartbeat may still update).
- **Sticky limit (R1):** keep last limit / lights / TBT until the next change. Timeouts do **not** clear the snapshot.
- **TBT distance:** tighten `speedTarget` near turns / forks / exits.
- **Green wave / SDI:** not in scope (no phone uplink; device ignores).
- **Road limit:** BLE reports raw `nRoadLimitSpeed` for HUD; execution = raw + device offset with a **usual floor of 60 km/h**. Invalid limit → do not invent; no snapshot → follow lead / model.
- **Traffic lights:** red/yellow aggressive decel toward stop (`accelTarget≈-2`); yellow near-distance treated like red; no fake green. Stay stopped through the APK countdown (`remainS==1` still stop). Planner holds at standstill until green; a flickering lead does not release a red stop. `remainS` / lamp color / distance are **HUD-only** (C3XL bar + cluster Kreuzung). They are not a go gate. IQ-link off still shows a plain stop-ahead cue from the vision model.
- **Lane B (gate only):** `KEY_TYPE=13012` → `laneRecommend`; `straight` suppresses auto lane-change desire. No lane-change HUD.
- **Cruise UI:** product max set speed `V_CRUISE_PRODUCT_MAX_KPH=120`.

## BLE GATT

Do **not** use setup UUIDs `73f2c700-…` (Konn3kt setup only).

| Role | UUID | Flags |
|------|------|-------|
| Service | `73f2c710-5e40-4d0d-8b7f-fde61f729100` | primary |
| Param write | `73f2c711-5e40-4d0d-8b7f-fde61f729100` | write, write-without-response |
| Status notify | `73f2c712-5e40-4d0d-8b7f-fde61f729100` | notify |

### Params (UI)

| Key | Meaning |
|-----|---------|
| `IqlinkEnabled` | Bluetooth toggle (product default on) |
| `IqlinkBlePsk` | Fixed 6-digit pair code `999999` (not shown in settings) |
| `IqlinkBleDiscovering` | Discovery window (legacy name; not for showing PSK) |
| `IqlinkBleLinkState` | `0` off / `1` connecting / `2` HMAC connected |
| `IqlinkBleConnected` | Mirror of `LinkState==2` |
| `IqlinkBlePeerConnected` | SoftBus Device1 Connected (status only; **not** green) |
| `IqlinkBlePairFailed` | Transient failure; device retries |

Top-level BT light: off=red; waiting/retry=yellow; **HMAC LinkState=2 only=green**.

### Envelope (UTF-8 JSON)

Prefer one ATT write after MTU negotiation. If fragmented, reassemble the same bytes then parse:

```json
{"v":1,"seq":123,"ts":1720000000000,"data":{...carrot fields...},"hmac":"<32 lowercase hex chars>"}
```

- `data`: flat Carrot dict with at least `nRoadLimitSpeed` (or `rgdata` wrapper).
- HMAC-SHA256 truncated to 32 hex:

```text
HMAC-SHA256(psk_utf8_bytes, f"{seq}:{ts}:".encode() + canonical_json(data)).hexdigest()[:32]
canonical_json = json.dumps(data, ensure_ascii=True, sort_keys=True, separators=(",",":"), allow_nan=False)
```

### Reject

- Missing / invalid PSK
- Bad HMAC
- `|now_ms - ts| > 120000`
- Seq replay within window 128

### PSK

- Params key `IqlinkBlePsk` (STRING), exactly 6 digits, fixed to `999999`
- Never log the full PSK (mask in cloudlog)

---

## 中文摘要

- **产品**：车上无导航会话；IQ-link只推即时参数；停导不清参；已连 = 设备 `LinkState=2`。
- **传输**：仅 BLE GATT；7705/7706/7713 已废弃。
- **纵向**：变更驱动 + 粘限速；限速原值上报、执行侧常保底 60；红黄猛减速，倒计时到 1 仍停；停稳后锁住到绿灯；无绿波/SDI；车道 B 仅直行门控。
- **PSK**：固定 `999999`，设置页不显示。
- **契约细节以上方英文为准**；字段以实现 `protocol.py` 为准。
