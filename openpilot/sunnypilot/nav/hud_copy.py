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
METERS = "米"
SECONDS = "秒"
LANE_LEFT = "靠左车道"
LANE_RIGHT = "靠右车道"
TURN_LEFT = "前方左转"
TURN_RIGHT = "前方右转"
DETAIL_SEP = " · "


def overlay_font_chars() -> str:
  return "".join((
    STOP_RED, STOP_YELLOW, STOP_GREEN, STOP_AHEAD, GO_AHEAD, WATCH_AHEAD,
    FOLLOW_LEAD, NO_SIGNAL, WAIT_DETECT, WAIT_PAIR, METERS, SECONDS,
    LANE_LEFT, LANE_RIGHT, TURN_LEFT, TURN_RIGHT, DETAIL_SEP,
  ))
