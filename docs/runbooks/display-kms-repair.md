# Display: restore full KMS and the native Wayland display (PR-1980)

## Symptom

- `ros-display` restarts endlessly (`docker inspect` shows a rising `RestartCount`).
- Its log shows `Gtk couldn't be initialized` and, before this fix, `process has finished cleanly`.
- On the host, `lightdm.service` has failed, there is no `labwc` process and no
  `/run/user/1000/wayland-0`. `~/.xsession-errors` contains
  `Found 0 GPUs, cannot create backend` / `Failed to open any DRM device`.
- `/boot/firmware/config.txt` contains `dtoverlay=vc4-fkms-v3d`.

Earlier versions of `setup/installation_scripts/set_system_settings.sh` rewrote the shipped
`dtoverlay=vc4-kms-v3d` to the legacy `vc4-fkms-v3d` on every setup run. On a Raspberry Pi 5
the kernel then bound the firmware KMS backend and reported no display, so the compositor
could not start.

## What the fix does

- Setup no longer writes `vc4-fkms-v3d`. Active `dtoverlay=vc4-fkms-v3d` lines are turned back
  into `dtoverlay=vc4-kms-v3d` in place: same `config.txt` section, overlay options kept
  (`,cma-256` etc.). Comments are not touched. If no `vc4-kms-v3d` overlay exists at all, one
  is appended. A file that already has full KMS is not changed and no backup is written.
- Before changing the file, it is copied to `/boot/firmware/config.txt.pib-backup-<timestamp>`.
- The 1024x600 panel directives (`hdmi_group`, `hdmi_mode`, `hdmi_cvt`, ...) are left as they
  were. Under full KMS with the shipped `disable_fw_kms_setup=1` the kernel takes the mode from
  the panel's EDID; no `video=` mode and no connector are forced.
- The same function runs in every full setup, so a later setup run keeps the repair.
- `ros-display` now waits for a compositor that accepts connections before GTK is loaded
  (capped backoff 1 s to 30 s, `PIB_DISPLAY_WAYLAND_WAIT_SECONDS`, default 600). It logs
  `waiting for the host Wayland session` with the reason (no runtime directory, no socket,
  stale socket, permission denied). After the budget it exits with status 75, so Docker starts
  it again and re-reads the runtime directory. `/pib/display_ready` = `ready` is published only
  after the GTK window has a Wayland surface and the compositor reports an output. A GTK
  failure ends the process with status 1, and `ros2 launch` then exits 1 instead of 0.

## Repair of an installed robot

Run everything as user `pib` (never as root); the code lives in `/home/pib/app`.

1. Bring the checkout to the merged revision and record the state before the repair:

   ```bash
   cd /home/pib/app/pib-backend
   git pull --ff-only
   git rev-parse HEAD
   grep -n 'vc4-' /boot/firmware/config.txt
   systemctl status lightdm --no-pager
   docker inspect -f '{{.Image}} {{.RestartCount}} {{.State.ExitCode}}' multirepo-ros-display-1
   ```

2. Repair the boot configuration. Only the display part of "Adjust system settings" runs:

   ```bash
   bash setup/setup-pib.sh --repair-display
   ```

   It prints `Restored full KMS (dtoverlay=vc4-kms-v3d) in /boot/firmware/config.txt; previous
   file: /boot/firmware/config.txt.pib-backup-<timestamp>. Reboot pib to apply it`. A second
   run changes nothing and writes no backup. It refuses root and systems other than
   Raspberry Pi OS bookworm/trixie with exit status 1.

3. Rebuild and recreate the display container with the new image:

   ```bash
   sudo docker compose -f /home/pib/app/pib-backend/docker-compose.yaml --profile all \
     up -d --build --force-recreate ros-display
   ```

4. Reboot (for the acceptance test: power off and cold boot):

   ```bash
   sudo reboot
   ```

## Checks after the reboot

```bash
grep -n 'vc4-' /boot/firmware/config.txt
systemctl status lightdm --no-pager
pgrep -a labwc
ls -l /run/user/1000/wayland-0
kmsprint || modetest -c          # connector, connected state and the selected mode
WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000 wlr-randr
docker inspect -f '{{.Image}} {{.RestartCount}} {{.State.ExitCode}}' multirepo-ros-display-1
docker logs --since 15m multirepo-ros-display-1
```

Expected: one active `dtoverlay=vc4-kms-v3d`, LightDM active, labwc running, the socket
present, an HDMI connector connected with a mode (expected 1024x600 from the panel's EDID; it
has not been measured yet), the restart counter unchanged over at least 10 minutes, and the
log line `GTK window realized on Wayland (1 output(s)); renderer ready` followed by
`published /pib/display_ready: ready`. Then show an expression and a text, hide them, and open
and close the host Chromium surface. Check touch input on the panel.

## Rollback

```bash
sudo cp /boot/firmware/config.txt.pib-backup-<timestamp> /boot/firmware/config.txt
sudo reboot
```

The repair does not change network or SSH configuration. If the graphics change prevents
normal boot or remote access, restore the saved boot file using local console/SD-card access.
The next setup or `--repair-display` run restores full KMS again.
