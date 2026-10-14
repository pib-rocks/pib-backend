# pib Tinkerforge RGB LED Button Service

## Ziel

Dieses ROS2-Paket bindet drei Tinkerforge RGB LED Button Bricklets ein und stellt sie für Blockly über Services bereit.

## Hardware-Konfiguration

Die Button-UIDs kommen von `GET /bricklet/connected`. Es zählen nur RGB LED Button Bricklets, und `button_id` 1..3 folgt der Port-Reihenfolge.

```bash
TF_HOST=localhost
TF_PORT=4223
FLASK_API_BASE_URL=http://flask-app:5000
```

In Docker ist `TF_HOST` abhängig davon, wo `brickd` läuft:

- `localhost`, wenn `brickd` im selben Container/Netzwerk-Namespace läuft
- IP des Hosts oder Docker-Gateway, wenn `brickd` auf dem Host läuft

## Start

```bash
source /opt/ros/jazzy/setup.bash
source /app/ros2_ws/install/setup.bash
ros2 run button_service button_service_node
```

## Services

```bash
/tf_button/set_color
/tf_button/read
/tf_button/wait
```

Button IDs sind `1`, `2`, `3` in Port-Reihenfolge der angeschlossenen RGB LED Button Bricklets.
