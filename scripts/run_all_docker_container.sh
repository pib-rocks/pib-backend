#!/bin/bash

# Farben für bessere Lesbarkeit
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Funktion für Fehlerbehandlung
error_exit() {
    echo -e "${RED}[ERROR] $1${NC}" >&2
    exit 1
}

# Prüfe, ob Docker läuft
if ! command -v docker &> /dev/null; then
    error_exit "Docker ist nicht installiert oder nicht im PATH."
fi

if ! sudo docker info &> /dev/null; then
    error_exit "Docker läuft nicht. Bitte starte Docker zuerst."
fi

# Funktion zum Starten eines Docker-Compose-Projekts
start_docker_compose() {
    local dir=$1
    local name=$2
    echo -e "${YELLOW}[INFO] Starte $name in $dir...${NC}"
    (
        cd "$dir" || error_exit "Verzeichnis $dir nicht gefunden."
        if ! sudo docker compose --profile "*" up --remove-orphans; then
            error_exit "Fehler beim Starten von $name."
        fi
    ) &
}

# Starte beide Projekte parallel
start_docker_compose "/home/pib/app/pib-backend" "pib-backend"
start_docker_compose "/home/pib/app/cerebra" "cerebra"

# Warte auf alle Hintergrundprozesse
wait
echo -e "${GREEN}[SUCCESS] Alle Docker-Container wurden erfolgreich gestartet.${NC}"
