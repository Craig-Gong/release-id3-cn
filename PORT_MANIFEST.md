# PORT_MANIFEST — `dev-sp`（本车适配叠 tip AccelBoost / chestnut）

| 项 | 值 |
|---|---|
| **Git 分支** | **`dev-sp`**（由 `dev-sp-id3` 改名） |
| 本机 worktree 目录 | `dev-sp-id3/`（含 submodule，目录暂未改名） |
| 底座 tip | `origin/dev-sp` @ **`da86ebd1e`** |
| 日常对照 | `dev-sp-egpu` @ **`389fff4ca`**（**勿冲、日常仍用这棵**） |
| 日常平台 | AGNOS **19.6** + dock **`ed4e39b7`** |
| 实验目标 | AGNOS **20.1** + dock **`8516b768`**（无安装器不刷） |
| 预刷审计 | **2026-11-01 完成** |

---

## 审计原则

1. **tip 已有新栈 → 优先用 tip**，不要用日常旧实现覆盖。  
2. **本车独有产品面** → 从日常移植。  
3. **日常 C3XL 补丁** → tip 缺口明确时才薄补。

---

## 刷前必修（已落地）

| 问题 | 风险 | 处置 |
|---|---|---|
| 日常 experimental planner/long_mpc 覆盖 tip | Experimental 与 tip `legacy_mpc` 不兼容 | **已回 tip** |
| 日常旧 ALC 覆盖 tip | 丢 edge/`lane_change_blocked` | **已回 tip ALC**；desire 四参对齐 |
| **`LongitudinalPlannerMode` 默认 1** | tip Experimental **不调** `apply_stop_helpers` → CTM/CUT/红钉全失效 | **默认改 0（Official）**；Legacy 路径**补** `apply_stop_helpers` |
| 车上残留 Mode=1 / Active=Experimental | 仍无 AccelBoost / 空路 soft cap | **`session._prefer_official`**：仅 Experimental→Official，并 heal Mode+Active；**不**锁 TN_NO_DEC |
| Official AccelBoost 误开 DEC | 与 tip Official 行为漂移 | Official **`enable_dec=False`** |
| junction 缺 `egpu_status` / 无 `usbgpu_*` | UI 崩 | 已补 + 别名 |
| HUD 替换 tip TrafficControl | Tesla 灯牌丢 | **并存** TC + Junction |
| tip Blendful/developer_ui/SCC 被改 | 冲 tip AccelBoost UI | **已回 tip** |
| modeld 无 `path_x/y` | 45 下打灯分不清转弯/变道 | **已接** |
| 残留 cruise_obstacle.py | 脏文件 | **已删** |
| Nav LC SET 未吞键 | 开 NavLcConfirm 时 SET 改 MAX / 误 enable / SLA | **已接** cruise_ext / car_events / SLA |
| tip 无 IQ-link MAX 跟路限 | 仪表 MAX 不跟高德；Gas Sync 无 override latch | **已从日常移植** cruise_ext + cruise Gas Sync |
| tip VW SET 吃 40 地板 | 低速 SET 显示假 MAX | **`volkswagen_standby_set_speed`**：跟当前车速 |
| tip params 无预编译兜底 | rsync 新 key → UnknownKeyName 崩 UI | **`_PREBUILT_KEY_DEFAULTS` + file-backed** |

## tip 上游功能（保留确认）

| tip 面 | 状态 |
|---|---|
| AccelBoost Official planner | 保留；叠 helpers + 空路 soft cap |
| Experimental/`legacy_mpc`/tuning | **与 tip 一致**（+ helpers 尾钩） |
| ALC edge / `lane_change_blocked` | tip 文件未改 |
| chestnut modeld_v2 / wait_for_chestnut / BIG_MODEL 60 | tip；仅 PPT100 setdefault |
| TrafficControl / Blendful / SCC UI / developer_ui | tip |
| models_manager / modeld_warmup / locationd_llk | tip process 在 |
| Tesla trafficcontrold | tip 代码在；本车 **should_run=False** |

## 本车移植面

| 面 | 说明 |
|---|---|
| ID.3 opendbc | E9/LSV；meb_pid_hold；cluster HUD 9/3 |
| pandad 禁内部 DFU | |
| C3XL 校准无 OnroadCycle | |
| IQ-link nav UDP :17710 | BLE 忽略 |
| CUT/CTM/NavLC/turn_prep/Lead | Official + Legacy 均 `apply_stop_helpers` |
| EcoFlow / lane_type | 默认关 ONNX |
| junction + egpu_hud | |

---

## 本地验证（本机）

```
python3 tools/id3_preflight_audit.py   # PREFLIGHT OK
# ecoflow venv + PYTHONPATH=.:opendbc_repo；跳过需整机 cereal/zstandard 的 DesireHelper 门闩 2 例
**305 passed, 2 skipped**（含 Official latch、IQ-link MAX、NavLC、CUT/CTM/Lead*…）
```

覆盖：CUT / CTM / NavLC / Lead* / standstill / nav_cruise / turn_prep / ecoflow / lane_type / hud_layout / egpu_hud / session Official latch / **IQ-link MAX**。

（需 cereal/capnp / tip ALC 整机测：车上或完整 OP venv。）

---

## 刷车 SOP（实验机；日常车勿用）

1. 确认不是日常 `dev-sp-egpu` 树；本机 `dev-sp` @ `da86ebd1e` + 本清单改动。  
2. `python3 tools/id3_preflight_audit.py` → OK。  
3. rsync **不要** `--delete` 整树；至少：`openpilot/sunnypilot/`、`openpilot/selfdrive/` 相关、`opendbc_repo/opendbc/car/volkswagen/`、`openpilot/common/params_keys.h`、`process_config.py`、`pandad.py`。  
4. 熄火→READY（插着 SuperSpeed **不要** reboot）。  
5. 路上验收：ID.3 / IQ-link UDP 绿 / Official AccelBoost+红停 / 无内部 panda DFU。  
6. **8516 / AGNOS 20.1：** 无可装包 + ed4e 回滚 → **不刷固件、不上路大模型**。

---

## 刷车后预期（软件面）

1. **默认 Official**：AccelBoost + CTM/CUT/红钉/空路 soft cap（残留 Mode=1 会被 latch heal）  
2. 人为切 Experimental：下次 onroad latch 仍回 Official；勿指望 Mode=1 常驻（TN_NO_DEC 不锁）  
3. ID.3 认车；无内部 panda DFU  
4. IQ-link UDP 绿；**MAX 跟路限** + Gas Sync；红停 / remainS / RTOR / 跟车门闩  
5. Nav LC Confirm 默认关；开时 SET 吞键；无 EA_02 自动打灯  
6. VW SET = 当前车速（无 40/105 假地板）  
7. turn_prep + desire 45；Blinker Pause 不挡低速转弯  
8. EcoFlow 默认关；SuperSpeed 不瞎 cycle  
9. eGPU tip chestnut；bigModelFailed toast-only  
10. 校准重置不 OnroadCycle；params 新 key rsync 有兜底 

**车上残留 Mode=1：** 无需手清——`latch_active_backend` 写回 Official(0)。

## 固件门闩

无 8516 可装包 + ed4e 回滚 → **不刷大模型固件、不上路大模型**。日常车继续 `dev-sp-egpu`。

## 明确不做

- egpu_loader 回迁；恢复自动打灯 / NavExit ALC  
- 把实验当日常；升 AGNOS 19.7；推 onemiless origin  
