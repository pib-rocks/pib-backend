from pathlib import Path
import re

p = Path("/root/app/pib-backend/docker-compose.yaml")
t = p.read_text()

ros_display = '''  ros-display:
    image: ros-display
    privileged: true
    build:
      context: .
      dockerfile: ./ros_packages/display/Dockerfile
    profiles:
      - all
      - display
    volumes:
      - /root/app/pib-expression-faces:/app/pib-expression-faces:ro
      - /tmp/.X11-unix:/tmp/.X11-unix
    environment:
      - STATIC_IMAGE_DIR=/app/ros2_ws/display/static_images
      - PIB_EXPRESSION_DIR=/app/pib-expression-faces
      - PIB_DISPLAY_TEXT_MAX_CHARS=40
      - PIB_DISPLAY_POLL_MS=5
      - PIB_DISPLAY_IDLE_SECONDS=10
      - DISPLAY=:0.0
    command: ros2 launch display launch.py
    restart: always
    networks:
      - pib-network

'''

# alten ros-display block ersetzen
t = re.sub(
    r"  ros-display:\n.*?(?=\n  ros-audio-io:)",
    ros_display,
    t,
    flags=re.S,
)

# falls pib-expression-manager noch drin ist, entfernen
t = re.sub(
    r"\n  pib-expression-manager:\n.*?(?=\n  [a-zA-Z0-9_-]+:|\nvolumes:)",
    "\n",
    t,
    flags=re.S,
)

# X11 Tippfehler global reparieren
t = t.replace("/tmp/.X11-unix:/tmp/.X11-uni ", "/tmp/.X11-unix:/tmp/.X11-unix")
t = t.replace("/tmp/.X11-unix:/tmp/.X11-uni", "/tmp/.X11-unix:/tmp/.X11-unix")

# networks/volumes sicherstellen
if "volumes:\n  programs:" not in t:
    t = t.rstrip() + "\n\nvolumes:\n  programs:\n"

if "networks:\n  pib-network:" not in t:
    t = t.rstrip() + "\n\nnetworks:\n  pib-network:\n    driver: bridge\n"

p.write_text(t)
print("docker-compose.yaml fixed: ros-display + audio preserved")
