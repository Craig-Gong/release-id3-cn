# Xiaoge lane.onnx（虚/实线）

权重约 13 MB，**不要** commit 进公开仓。

```bash
# 车上
bash /data/openpilot/../../demos/lane-type/fetch_lane_onnx.sh
# 或本机拉完再 scp：
./demos/lane-type/fetch_lane_onnx.sh /tmp/lane.onnx
scp /tmp/lane.onnx c3xl:/data/media/0/models/lane.onnx
```

车上路径固定：`/data/media/0/models/lane.onnx`（可用 `LANE_ONNX_PATH` 覆盖）。

设置：Steering → Customize Lane Change → **Lane Type Assist**（`LaneTypeOnnx`，默认关）。
开后：≥45 km/h 拨杆变道遇高置信实线硬挡；未知/过期 fail-open。
