"""On-road overlay copy. Keep these in the unifont atlas or Raylib draws '?'."""

STOP_RED = "红灯"
STOP_YELLOW = "黄灯"
STOP_GREEN = "绿灯"
STOP_AHEAD = "前方停车"
GO_AHEAD = "可通行"
WATCH_AHEAD = "注意前方"
FOLLOW_LEAD = "跟前车"
NO_SIGNAL = "暂无信号"
WAIT_DETECT = "等待识别"
WAIT_PAIR = "等待配对"
# ASCII units — zh-CHS Noto fallback often lacks 米/公里 and draws '?'.
METERS = "m"
SECONDS = "s"
KILOMETERS = "km"
MINUTES = "min"
LANE_LEFT = "靠左车道"
LANE_RIGHT = "靠右车道"
TURN_LEFT = "前方左转"
TURN_RIGHT = "前方右转"
STRAIGHT_AHEAD = "前方直行"
STRAIGHT_LANE = "直行车道"
EXIT_AHEAD = "前方驶出"
ARRIVE_SOON = "即将到达"
NAV_EMPTY = "暂无导航推送"
NAV_GUIDE = "路况引导"
LANE_GUIDE = "车道引导"
TURN_HINT = "转向提示"
STRAIGHT_HINT = "直行提示"
EXIT_HINT = "驶出提示"
ARRIVE_HINT = "到达提示"
ENTER_PREFIX = "进入 · "
REMAIN_DIST = "路程剩余："
REMAIN_TIME = "时间剩余："
NAV_GOAL = "导航目的地："
DETAIL_SEP = " · "

# eGPU strip (same Noto fallback path as nav HUD — must be in glyph set).
EGPU_HEAD = "大模型"
EGPU_DETAIL = (
  "未连接未编译加载已回退小模型等待启动运行中USB降速遥测暂无"
)
EGPU_DC = "开关未启用未知"


def overlay_font_chars() -> str:
  return "".join((
    STOP_RED, STOP_YELLOW, STOP_GREEN, STOP_AHEAD, GO_AHEAD, WATCH_AHEAD,
    FOLLOW_LEAD, NO_SIGNAL, WAIT_DETECT, WAIT_PAIR, METERS, SECONDS, KILOMETERS, MINUTES,
    LANE_LEFT, LANE_RIGHT, TURN_LEFT, TURN_RIGHT, STRAIGHT_AHEAD, STRAIGHT_LANE,
    EXIT_AHEAD, ARRIVE_SOON, NAV_EMPTY, NAV_GUIDE, LANE_GUIDE, TURN_HINT,
    STRAIGHT_HINT, EXIT_HINT, ARRIVE_HINT, ENTER_PREFIX, REMAIN_DIST, REMAIN_TIME,
    NAV_GOAL, DETAIL_SEP, EGPU_HEAD, EGPU_DETAIL, EGPU_DC,
  ))
