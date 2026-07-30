from flask import Blueprint, jsonify, request
from service import marimo_service

marimo_bp = Blueprint("marimo", __name__)


@marimo_bp.route("/notebooks", methods=["GET"])
@marimo_bp.route("/v1/marimo/notebooks", methods=["GET"])
def get_all_notebooks():
    notebooks = marimo_service.list_notebooks()
    return jsonify({"status": "success", "notebooks": notebooks}), 200


@marimo_bp.route("/notebooks", methods=["POST"])
@marimo_bp.route("/v1/marimo/notebooks", methods=["POST"])
def create_notebook():
    data = request.get_json() or {}
    name = data.get("name")
    content = data.get("content")
    if not name:
        return jsonify({"status": "error", "message": "Notebook name is required."}), 400

    code, res = marimo_service.create_notebook(name, content)
    return jsonify(res), code


@marimo_bp.route("/notebooks/<name>", methods=["GET"])
@marimo_bp.route("/v1/marimo/notebooks/<name>", methods=["GET"])
def get_notebook(name):
    code, res = marimo_service.get_notebook(name)
    return jsonify(res), code


@marimo_bp.route("/notebooks/<name>/rename", methods=["POST", "PUT"])
@marimo_bp.route("/v1/marimo/notebooks/<name>/rename", methods=["POST", "PUT"])
def rename_notebook(name):
    data = request.get_json() or {}
    new_name = data.get("newName")
    if not new_name:
        return jsonify({"status": "error", "message": "New notebook name is required."}), 400

    code, res = marimo_service.rename_notebook(name, new_name)
    return jsonify(res), code


@marimo_bp.route("/notebooks/<name>", methods=["DELETE"])
@marimo_bp.route("/v1/marimo/notebooks/<name>", methods=["DELETE"])
def delete_notebook(name):
    code, res = marimo_service.delete_notebook(name)
    return jsonify(res), code
