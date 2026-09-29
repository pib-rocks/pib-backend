#!/bin/bash
#
# Narrow root helper for changing systemd's existing hardware-watchdog timeout.
# This file must stay self-contained: importing code from the pib-writable checkout
# would turn that code into a root privilege-escalation path.

set -Eeuo pipefail

readonly PIB_UPDATE_DIR="/home/pib/app/.update"
readonly TARGET_FILE="$PIB_UPDATE_DIR/watchdog.target"
readonly MAX_TARGET_US=1800000000

if [ "$#" -ne 0 ]; then
    printf 'This helper accepts no arguments\n' >&2
    exit 2
fi

if [ ! -f "$TARGET_FILE" ] || [ -L "$TARGET_FILE" ]; then
    printf 'Invalid watchdog target file\n' >&2
    exit 1
fi

target="$(cat "$TARGET_FILE")"

# Deliberately duplicate the unprivileged module's validation at the privilege
# boundary. Only an integer in systemd's accepted microsecond range may reach busctl.
if [[ ! "$target" =~ ^[0-9]+$ ]]; then
    printf 'Invalid watchdog target value\n' >&2
    exit 1
fi
normalized="$target"
while [ "${#normalized}" -gt 1 ] && [ "${normalized#0}" != "$normalized" ]; do
    normalized="${normalized#0}"
done
if [ "${#normalized}" -gt 10 ] \
    || { [ "${#normalized}" -eq 10 ] && [[ "$normalized" > "$MAX_TARGET_US" ]]; }; then
    printf 'Invalid watchdog target value\n' >&2
    exit 1
fi

busctl set-property org.freedesktop.systemd1 /org/freedesktop/systemd1 \
    org.freedesktop.systemd1.Manager RuntimeWatchdogUSec t "$normalized"
printf 'Applied systemd RuntimeWatchdogUSec=%s\n' "$normalized"
