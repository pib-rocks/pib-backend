#!/bin/bash

resolve_hardware_variant() {
  local resolved_variants=()
  local argument

  for argument in "$@"; do
    case "$argument" in
      --pib4edu)
        resolved_variants+=("pib4edu")
        ;;
      --pib4advanced)
        resolved_variants+=("pib4advanced")
        ;;
      --pib5advanced)
        resolved_variants+=("pib5advanced")
        ;;
      --pib5museum)
        resolved_variants+=("pib5museum")
        ;;
      *)
        echo "Unknown hardware variant flag: ${argument}" >&2
        return 2
        ;;
    esac
  done

  if [ "${#resolved_variants[@]}" -gt 1 ]; then
    echo "Multiple hardware variant flags provided: ${resolved_variants[*]}" >&2
    return 2
  fi

  if [ "${#resolved_variants[@]}" -eq 0 ]; then
    echo "pib5edu"
  else
    echo "${resolved_variants[0]}"
  fi
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  resolve_hardware_variant "$@"
  exit $?
fi
