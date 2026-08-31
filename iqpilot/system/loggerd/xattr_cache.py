import errno
import os

import xattr

_cached_attributes: dict[tuple[str, str], tuple[tuple[int, int, int], bytes | None]] = {}

def getxattr(path: str, attr_name: str) -> bytes | None:
  key = (path, attr_name)
  st = os.stat(path)
  identity = (st.st_dev, st.st_ino, st.st_ctime_ns)
  cached = _cached_attributes.get(key)
  if cached is None or cached[0] != identity:
    try:
      response = xattr.getxattr(path, attr_name)
    except OSError as e:
      # ENODATA (Linux) or ENOATTR (macOS) means attribute hasn't been set
      if e.errno == errno.ENODATA or (hasattr(errno, 'ENOATTR') and e.errno == errno.ENOATTR):
        response = None
      else:
        raise
    _cached_attributes[key] = (identity, response)
  return _cached_attributes[key][1]

def setxattr(path: str, attr_name: str, attr_value: bytes) -> None:
  xattr.setxattr(path, attr_name, attr_value)
  st = os.stat(path)
  _cached_attributes[(path, attr_name)] = ((st.st_dev, st.st_ino, st.st_ctime_ns), attr_value)
