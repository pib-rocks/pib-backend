#!/bin/bash

# Verzeichnis für das Skript erstellen, falls es nicht existiert
mkdir -p /home/pib/app/pib-backend/scripts

# Host-System aktualisieren
echo "Aktualisiere das Host-System..."
sudo apt update && sudo apt upgrade -y && sudo apt autoremove -y

# Container multirepo-sqlite-viewer-1 stoppen und entfernen, falls er existiert
sqlite_viewer="multirepo-sqlite-viewer-1"

if sudo docker ps -a --format '{{.Names}}' | grep -q "^$sqlite_viewer$"; then
  echo "Container $sqlite_viewer existiert. Stoppe und entferne ihn..."
  sudo docker stop "$sqlite_viewer"
  sudo docker rm "$sqlite_viewer"
else
  echo "Container $sqlite_viewer existiert nicht. Überspringe diesen Schritt."
fi

# Container-IDs oder Namen für die Aktualisierung
containers=(
  "multirepo-angular-app-1"
  "multirepo-ros-motors-1"
  "multirepo-ros-voice-assistant-1"
  "multirepo-ros-programs-1"
  "multirepo-tinkerforge-brickd-1"
  "multirepo-flask-app-1"
  "multirepo-ros-camera-1"
  "multirepo-pib-blockly-server-1"
  "multirepo-rosbridge-ws-1"
  "multirepo-ros-display-1"
)

# Aktualisierung jedes Containers
for container in "${containers[@]}"; do
  echo "Aktualisiere Container: $container"
  sudo docker exec -it "$container" bash -c "apt update && apt upgrade -y && apt autoremove -y"
done

# System neu starten
echo "System wird neu"
sudo reboot now
