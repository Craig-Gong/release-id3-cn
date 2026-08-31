#!/usr/bin/env python3
from iqpilot.common.params import Params
print(Params().get("IqlinkBleLinkState"))
print(Params().get_bool("IqlinkBleConnected"))
print(Params().get_bool("IqlinkBlePeerConnected"))
