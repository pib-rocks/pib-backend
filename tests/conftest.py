import sys
import os

FLASK_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "pib_api", "flask")
)
if FLASK_DIR not in sys.path:
    sys.path.insert(0, FLASK_DIR)
