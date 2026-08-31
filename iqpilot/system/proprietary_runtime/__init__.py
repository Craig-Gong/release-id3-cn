import os

from iqpilot.common.basedir import BASEDIR


os.environ.setdefault("OPENPILOT_BASEDIR", BASEDIR)
os.environ.setdefault("IQPILOT_PROPRIETARY_ROOT", os.path.join(BASEDIR, ".iqpilot", "bundles"))

# _verified_import.so ships ONLY in the signed rootfs runtime. It MUST be prepended so a
# repo-side _verified_import.py can never shadow the signed native module via __path__
# order. runtime_paths.py lives in the repo copy and still resolves from the fall-through.
_ROOTFS_PKG = "/usr/libexec/iqpilot/python/openpilot/system/proprietary_runtime"
if os.path.isdir(_ROOTFS_PKG):
  if _ROOTFS_PKG in __path__:
    __path__.remove(_ROOTFS_PKG)
  __path__.insert(0, _ROOTFS_PKG)
