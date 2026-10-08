#!/bin/bash
# Marker body for the installer flag --no-smart-chats.
# $1 is 0 when Hermes chats are disabled, anything else leaves Smart available.
# The runtime reads this text from /etc/pib_smart_chats.

smart_chats_marker() {
  if [ "${1:-1}" = "0" ]; then
    printf '%s\n' disabled
  else
    printf '%s\n' enabled
  fi
}
