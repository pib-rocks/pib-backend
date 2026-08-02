import os
import re
import logging
from typing import List, Dict, Any, Optional, Tuple

NOTEBOOKS_DIR = os.getenv("MARIMO_NOTEBOOKS_DIR", "/home/pib/programs/notebooks")

DEFAULT_DEMO_NOTEBOOK = """import marimo

__generated_with = "0.10.0"
app = marimo.App(width="medium")


@app.cell
def __():
    import marimo as mo
    return mo,


@app.cell
def __(mo):
    mo.md(
        r\"\"\"
        # 🤖 pib Robot SDK Demo Notebook
        Welcome to **Marimo** on pib! You can control pib's joints, motors, poses, and sensors interactively in Python.
        \"\"\"
    )
    return


@app.cell
def __():
    try:
        from pib_sdk import Pib
        pib = Pib()
        print("Successfully connected to pib SDK!")
    except Exception as e:
        print("SDK notice:", e)
    return Pib, pib


if __name__ == "__main__":
    app.run()
"""

# Fresh notebooks created via "New notebook" start with one blank cell.
EMPTY_NOTEBOOK = """import marimo

__generated_with = "0.10.0"
app = marimo.App(width="medium")


@app.cell
def __():
    return


if __name__ == "__main__":
    app.run()
"""


def _ensure_dir():
    os.makedirs(NOTEBOOKS_DIR, exist_ok=True)
    demo_path = os.path.join(NOTEBOOKS_DIR, "pib_sdk_demo.py")
    if not os.path.exists(demo_path):
        try:
            with open(demo_path, "w", encoding="utf-8") as f:
                f.write(DEFAULT_DEMO_NOTEBOOK)
        except Exception as e:
            logging.error(f"Failed to create default demo notebook: {e}")


def list_notebooks() -> List[Dict[str, Any]]:
    _ensure_dir()
    notebooks = []
    try:
        for fname in sorted(os.listdir(NOTEBOOKS_DIR)):
            if fname.endswith(".py"):
                fpath = os.path.join(NOTEBOOKS_DIR, fname)
                mtime = os.path.getmtime(fpath)
                size = os.path.getsize(fpath)
                notebooks.append({
                    "name": fname,
                    "title": fname.replace(".py", "").replace("_", " ").title(),
                    "path": fpath,
                    "updatedAt": mtime,
                    "sizeBytes": size
                })
    except Exception as e:
        logging.error(f"Error listing Marimo notebooks: {e}")
    return notebooks


def create_notebook(name: str, content: Optional[str] = None) -> Tuple[int, Dict[str, Any]]:
    _ensure_dir()
    if not name.endswith(".py"):
        name += ".py"
    clean_name = re.sub(r'[^a-zA-Z0-9_\-\.]', '_', name)
    fpath = os.path.join(NOTEBOOKS_DIR, clean_name)

    if os.path.exists(fpath):
        return 400, {"status": "error", "message": f"Notebook '{clean_name}' already exists."}

    initial_content = content if content else EMPTY_NOTEBOOK
    try:
        with open(fpath, "w", encoding="utf-8") as f:
            f.write(initial_content)
        return 201, {
            "status": "success",
            "notebook": {
                "name": clean_name,
                "path": fpath
            }
        }
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}


def get_notebook(name: str) -> Tuple[int, Dict[str, Any]]:
    _ensure_dir()
    if not name.endswith(".py"):
        name += ".py"
    fpath = os.path.join(NOTEBOOKS_DIR, name)
    if not os.path.exists(fpath):
        return 404, {"status": "error", "message": f"Notebook '{name}' not found."}

    try:
        with open(fpath, "r", encoding="utf-8") as f:
            content = f.read()
        return 200, {"status": "success", "name": name, "content": content}
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}


def rename_notebook(old_name: str, new_name: str) -> Tuple[int, Dict[str, Any]]:
    _ensure_dir()
    if not old_name.endswith(".py"):
        old_name += ".py"
    if not new_name.endswith(".py"):
        new_name += ".py"
    clean_new = re.sub(r'[^a-zA-Z0-9_\-\.]', '_', new_name)

    old_path = os.path.join(NOTEBOOKS_DIR, old_name)
    new_path = os.path.join(NOTEBOOKS_DIR, clean_new)

    if not os.path.exists(old_path):
        return 404, {"status": "error", "message": f"Notebook '{old_name}' not found."}
    if os.path.exists(new_path) and old_path != new_path:
        return 400, {"status": "error", "message": f"Target notebook '{clean_new}' already exists."}

    try:
        os.rename(old_path, new_path)
        return 200, {"status": "success", "oldName": old_name, "newName": clean_new}
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}


def delete_notebook(name: str) -> Tuple[int, Dict[str, Any]]:
    _ensure_dir()
    if not name.endswith(".py"):
        name += ".py"
    fpath = os.path.join(NOTEBOOKS_DIR, name)

    if not os.path.exists(fpath):
        return 404, {"status": "error", "message": f"Notebook '{name}' not found."}

    try:
        os.remove(fpath)
        return 200, {"status": "success", "message": f"Notebook '{name}' deleted successfully."}
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}
