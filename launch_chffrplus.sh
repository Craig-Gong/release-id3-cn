#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

export HOME="${HOME:-/home/comma}"
export IQPILOT_PROPRIETARY_ROOT="$DIR/artifacts"
source "$DIR/launch_env.sh"
export PATH="/usr/local/venv/bin:$PATH"

function agnos_version_allowed {
  local current_version="$1"
  local expected_version="$2"
  local compat_version

  if [ "$current_version" = "$expected_version" ] || [[ "$current_version" = "$expected_version"-* ]]; then
    return 0
  fi

  IFS=',' read -r -a compat_versions <<< "${AGNOS_COMPAT_VERSIONS:-}"
  for compat_version in "${compat_versions[@]}"; do
    compat_version="${compat_version// /}"
    if [ -n "$compat_version" ] && [ "$current_version" = "$compat_version" ]; then
      return 0
    fi
  done

  return 1
}

function set_lite_hw() {
  # Official IQ uses "tici DTB + i2c 0x10 missing" as C3 Lite. This car is a
  # Mr.One C3XL with the same tici DTB and u-blox GPS; that probe is a false
  # positive and would disable ubloxd. Never treat this fork as Lite.
  unset LITE
  rm -f /tmp/lite_hw
}

function install_iq_command() {
  local rc_file="$HOME/.bashrc"

  [ -w "$HOME" ] || return 0
  touch "$rc_file" 2>/dev/null || return 0
  local rc_temp="${rc_file}.iq-tmp.$$"
  awk '/^# >>> iq command/{skip=1} /^# <<< iq command/{skip=0; next} skip{next}
       !/^alias op=.*tools\/op\.sh/ && !/^alias iq=.*tools\/iq\.sh/' "$rc_file" > "$rc_temp" || return 0
  mv "$rc_temp" "$rc_file" || return 0
  cat >> "$rc_file" <<'IQFN'
# >>> iq command (auto-managed, do not edit) >>>
unalias iq op 2>/dev/null
iq() {
  local d="$PWD"
  while [ "$d" != "/" ]; do
    if [ -x "$d/iqpilot/tools/iq.sh" ] && { [ -f "$d/launch_iqpilot.sh" ] || [ -f "$d/launch_openpilot.sh" ]; }; then
      "$d/iqpilot/tools/iq.sh" "$@"; return
    fi
    d="$(dirname "$d")"
  done
  for d in /data/iqpilot /data/openpilot; do
    if [ -x "$d/iqpilot/tools/iq.sh" ] && { [ -f "$d/launch_iqpilot.sh" ] || [ -f "$d/launch_openpilot.sh" ]; }; then
      "$d/iqpilot/tools/iq.sh" "$@"; return
    fi
  done
  echo "iq: no IQ.Pilot checkout found (searched up from $PWD, then /data/iqpilot, /data/openpilot)" >&2
  return 1
}
# <<< iq command (auto-managed) <<<
IQFN
}

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # iptables-nft fails on this kernel (no nf_tables module), breaking NM hotspot NAT rules
  if [ "$(readlink /etc/alternatives/iptables)" != "/usr/sbin/iptables-legacy" ]; then
    sudo mount -o remount,rw /
    sudo ln -sf /usr/sbin/iptables-legacy /etc/alternatives/iptables
    sudo ln -sf /usr/sbin/iptables-legacy-restore /etc/alternatives/iptables-restore
    sudo ln -sf /usr/sbin/iptables-legacy-save /etc/alternatives/iptables-save
    sudo mount -o remount,ro /
  fi

  # Check if AGNOS update is required
  CURRENT_AGNOS_VERSION=$(< /VERSION)
  if ! agnos_version_allowed "$CURRENT_AGNOS_VERSION" "$AGNOS_VERSION"; then
    AGNOS_PY="$DIR/iqpilot/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/iqpilot/system/hardware/tici/agnos.json"
    if [ -f /sys/firmware/devicetree/base/model ]; then
      DEVICE_MODEL="$(tr -d '\0' </sys/firmware/devicetree/base/model)"
      case "$DEVICE_MODEL" in
        *"comma tici"*|*"comma three"*)
          # comma 3 keeps its stock 15.1 bootloaders: comma's 16 abl has a board-id
          # check that rejects comma 3 ("Unsupported firmware detected"). It still gets
          # the IQ boot (comma_tici.dtb) + IQ.OS 3.4 system via this hybrid manifest.
          MANIFEST="$DIR/iqpilot/system/hardware/tici/agnos_tici_15_1.json"
          ;;
      esac
    fi
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    # On stock AGNOS the updater picks updater_weston (weston is up), a compiled display binary that
    # can't get a compositor surface on a first boot, so the IQ.OS flash silently wedges on the comma
    # logo. There, flash headless via agnos.py --swap (identical download+write+swap, no surface).
    # On IQ.OS weston is down -> the updater picks updater_magic (DRM), which renders the on-screen
    # progress prompt fine, so fall through and keep it. This mirrors the updater's own weston check.
    # C3XL: agnos.py --swap and updater_magic loop forever if git.konn3kt.com/iqos
    # 404s (private repo / missing GitAuthBlob). Time-box the flash so first boot
    # still reaches manager on the current IQ.OS; updated.py can retry later.
    AGNOS_FLASH_TIMEOUT_S="${AGNOS_FLASH_TIMEOUT_S:-2700}"
    if systemctl is-active --quiet weston-ready; then
      if timeout "$AGNOS_FLASH_TIMEOUT_S" $AGNOS_PY --swap $MANIFEST; then
        sudo reboot
      fi
    elif timeout "$AGNOS_FLASH_TIMEOUT_S" "$DIR/iqpilot/system/hardware/tici/updater" $AGNOS_PY $MANIFEST; then
      sudo reboot
    fi
    echo "IQ.OS update to $AGNOS_VERSION did not finish; continuing on $CURRENT_AGNOS_VERSION"
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock
  for server_dir in "$HOME/.cursor-server" "$HOME/.windsurf-server" "$HOME/.vscode-server"; do
    [ -L "$server_dir" ] && unlink "$server_dir"
  done

  install_iq_command

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # Python env + proprietary runtime sync. Lives in artifacts/runtime/env_sync.sh so updated.py
  # can run the identical steps at update-apply time (offroad, stack idle); by the
  # time this runs at boot it is normally a warm no-op. Real work still lands here
  # for local dev edits or a failed apply-time prep, under the boot spinner as before.
  source "$DIR/artifacts/runtime/env_sync.sh"
  sync_python_env || return 1
  export PATH="$DIR/.venv/bin:$PATH"

  RUNTIME_COMPAT_ROOT="$DIR/.iqpilot/runtime_root"
  mkdir -p "$RUNTIME_COMPAT_ROOT"
  ln -sfn "$DIR/iqpilot" "$RUNTIME_COMPAT_ROOT/iqpilot"
  ln -sfn iqpilot "$RUNTIME_COMPAT_ROOT/openpilot"
  ln -sfn "$DIR/iqpilot/system" "$RUNTIME_COMPAT_ROOT/system"

  ln -sfn $(pwd) /data/pythonpath
  export IQPILOT_SOURCE_ROOT="${IQPILOT_SOURCE_ROOT:-$DIR/iqpilot}"
  export PYTHONSAFEPATH=1
  VERIFIED_PYTHON_ROOT="/usr/libexec/iqpilot/python"
  IQPILOT_PYTHONPATH="$VENV_SITE_PACKAGES:$PWD"
  if [ -d "$PWD/artifacts/package_runtime" ]; then
    IQPILOT_PYTHONPATH="$PWD/artifacts/package_runtime:$IQPILOT_PYTHONPATH"
  fi
  if [ -d "$VERIFIED_PYTHON_ROOT" ]; then
    export PYTHONPATH="$VERIFIED_PYTHON_ROOT:$IQPILOT_PYTHONPATH"
  else
    export PYTHONPATH="$IQPILOT_PYTHONPATH"
  fi

  # Install independent systemd services BEFORE build so they run even
  # when openpilot fails to compile — SSH/BLE/hephaestusd must always
  # be reachable for device recovery.
  for service_name in hephaestusd ble-transportd flockd; do
    service_src="$DIR/iqpilot/system/${service_name}.service"
    service_dst="/etc/systemd/system/${service_name}.service"
    service_lib="/lib/systemd/system/${service_name}.service"
    service_dropin="/run/systemd/system/${service_name}.service.d"
    service_exec="$(grep '^ExecStart=' "$service_src")"
    sudo mkdir -p "$service_dropin"
    printf '[Service]\nWorkingDirectory=%s\nEnvironment="IQPILOT_SOURCE_ROOT=%s/iqpilot"\nEnvironment="IQPILOT_PROPRIETARY_ROOT=%s/artifacts"\nEnvironment="PYTHONPATH=/usr/libexec/iqpilot/python:%s:%s"\nExecStart=\n%s\n' "$RUNTIME_COMPAT_ROOT" "$DIR" "$DIR" "$VENV_SITE_PACKAGES" "$DIR" "$service_exec" | sudo tee "$service_dropin/iqpilot-packages.conf" >/dev/null
    sudo systemctl daemon-reload
    if [ -f "$service_lib" ] && grep -q "/usr/libexec/iqpilot/iqpilot_bundle_runner" "$service_lib"; then
      if [ -f "$service_dst" ]; then
        sudo mount -o remount,rw /
        sudo rm -f "$service_dst"
        sudo systemctl daemon-reload
        sudo mount -o remount,ro /
      fi
      sudo systemctl enable "${service_name}.service"
      # --no-block: these units ExecStartPre-wait for the verified runtime. Blocking here
      # made a missing runner stall the whole install for the unit's 600s timeout (x3 units).
      if systemctl is-active --quiet "${service_name}.service"; then
        sudo systemctl restart --no-block "${service_name}.service"
      else
        sudo systemctl start --no-block "${service_name}.service"
      fi
    elif [ -f "$service_src" ]; then
      if [ ! -f "$service_dst" ] || ! cmp -s "$service_src" "$service_dst"; then
        sudo mount -o remount,rw /
        sudo cp "$service_src" "$service_dst"
        sudo systemctl daemon-reload
        sudo systemctl enable "${service_name}.service"
        sudo mount -o remount,ro /
        sudo systemctl restart --no-block "${service_name}.service"
      elif ! systemctl is-active --quiet "${service_name}.service"; then
        sudo systemctl start --no-block "${service_name}.service"
      fi
    fi
  done

  # C3XL: clear any Lite false positive (do not probe i2c 0x10)
  set_lite_hw

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init

    sudo "$DIR/iqpilot/system/hardware/tici/zram_setup.sh" || true
  fi

  if [ -f "$DIR/artifacts/runtime/apply_boot_branding.py" ]; then
    sudo python3 "$DIR/artifacts/runtime/apply_boot_branding.py" || true
  fi

  # /home is an ephemeral overlay (resets each boot); re-inject the source line. best-effort
  if [ -f "$DIR/iqpilot/tools/iqpilot/git-pretty.sh" ] && [ -w "$HOME/.bashrc" ]; then
    grep -q 'iqpilot/git-pretty.sh' "$HOME/.bashrc" 2>/dev/null || \
      echo "[ -f $DIR/iqpilot/tools/iqpilot/git-pretty.sh ] && source $DIR/iqpilot/tools/iqpilot/git-pretty.sh" >> "$HOME/.bashrc" || true
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd "$DIR/iqpilot/system/manager"
  export PWD="$(pwd)"
  if [ -f "$DIR/prebuilt" ] && [ ! -x "$DIR/iqpilot/system/camerad/camerad" ]; then
    echo "ERROR: prebuilt tree is missing native camerad. rsync the Mac IQ.Pilot-on-release working tree; a git clone does not include gitignored aarch64 binaries."
  fi
  if [ ! -f $DIR/prebuilt ]; then
    if pkill -f /tmp/installer 2>/dev/null; then sleep 1; fi
    "$DIR/.venv/bin/python3" ./build.py
  fi

  if [ -x "$DIR/.venv/bin/python3" ]; then
    "$DIR/.venv/bin/python3" ./manager.py
  else
    echo "WARNING: $DIR/.venv/bin/python3 missing; falling back to /usr/local/venv/bin/python3"
    /usr/local/venv/bin/python3 ./manager.py
  fi

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
