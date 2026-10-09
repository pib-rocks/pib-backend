#!/bin/bash
#
# Sourced by setup/setup-pib.sh and setup/update-pib.sh. Defines functions
# only; sourcing this file does not write or restart anything.

# True when the ollama unit exists. The 4 GiB variants never install it
# (setup-pib.sh variant gate). Callers treat a missing unit as success and
# must not create the drop-in directory.
function ollama_unit_installed() {
  systemctl cat ollama >/dev/null 2>&1
}

# ensure_ollama_listen_dropin [restart]
#
# Writes the systemd drop-in that sets OLLAMA_HOST=0.0.0.0:11434.
# Containers reach the daemon at host.docker.internal, the bridge address.
# The official unit binds the loopback interface, so the flask tags probe
# fails and Cerebra never offers qwen-fast. PIB_OLLAMA_DROPIN points the
# unit tests at a scratch file.
#
# Idempotent. A drop-in that already matches is left in place: no rewrite,
# no daemon-reload, no restart. A second run on an already-configured
# device therefore does not touch the service.
#
# Apply mode, first argument:
#   (omitted)  Install path. Do not restart. setup-pib.sh calls this before
#              the first systemctl start, so that start loads the drop-in.
#              An already-active unit is left running and a warning is
#              printed. Restarting it would drop the local model during an
#              install that otherwise leaves a healthy service alone.
#   restart    Update path. A device that was installed before this drop-in
#              exists is already serving on loopback. Writing the file
#              without a restart changes nothing until the next reboot, so
#              the update would still look like it did nothing. Restart
#              when this call changed the file and the unit is active. The
#              outage is a few seconds inside the update, which is already
#              a maintenance window. A unit that is not active is not
#              started: a stopped ollama stays stopped, and the next start
#              loads the drop-in.
function ensure_ollama_listen_dropin() {
  local apply="${1:-defer}"
  local dropin="${PIB_OLLAMA_DROPIN:-/etc/systemd/system/ollama.service.d/override.conf}"
  local staged="/tmp/pib-ollama-dropin.$$"
  local already_active=0

  # Absolute paths: the unit tests run this function with a PATH that contains
  # only their stubs, and a bare mkdir/cmp would miss the real coreutils.
  printf '%s\n' '[Service]' 'Environment="OLLAMA_HOST=0.0.0.0:11434"' > "$staged" || return 1
  if [ -f "$dropin" ] && /usr/bin/cmp -s "$staged" "$dropin"; then
    /usr/bin/rm -f "$staged"
    print INFO "ollama: container listen address already configured"
    return 0
  fi
  if systemctl is-active --quiet ollama; then
    already_active=1
  fi
  sudo /usr/bin/mkdir -p "$(/usr/bin/dirname "$dropin")" || { /usr/bin/rm -f "$staged"; return 1; }
  sudo /usr/bin/tee "$dropin" >/dev/null < "$staged" || { /usr/bin/rm -f "$staged"; return 1; }
  /usr/bin/rm -f "$staged"
  sudo systemctl daemon-reload || return 1
  if [ "$apply" = "restart" ]; then
    if [ "$already_active" -eq 1 ]; then
      sudo systemctl restart ollama || return 1
      print INFO "ollama: restarted so OLLAMA_HOST=0.0.0.0:11434 is in effect"
    else
      print INFO "ollama: configured OLLAMA_HOST=0.0.0.0:11434; service is not active, so it was not restarted"
    fi
    return 0
  fi
  if [ "$already_active" -eq 1 ]; then
    print WARN "ollama: OLLAMA_HOST applies on the next start; this step does not restart a service that is already active"
  else
    print INFO "ollama: configured OLLAMA_HOST=0.0.0.0:11434"
  fi
}
