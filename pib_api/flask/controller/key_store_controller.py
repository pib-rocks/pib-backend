"""HTTP surface for the encrypted provider key store.

Responses name a wrong password and carry no secret material. The decrypted
keys stay in the key-store process.
"""

from flask import Blueprint, jsonify, request

from service import key_store_service

bp = Blueprint("key_store_controller", __name__)


def _payload() -> dict | None:
    body = request.get_json(silent=True)
    if not isinstance(body, dict):
        return None
    return body


def _text(body: dict, field: str) -> str | None:
    value = body.get(field)
    if not isinstance(value, str) or value == "":
        return None
    return value


def _failure(error: key_store_service.KeyStoreError):
    return (
        jsonify(
            {
                "successful": False,
                "credentials": [],
                "error": str(error),
            }
        ),
        error.status_code,
    )


def _bad_request():
    return (
        jsonify({"successful": False, "credentials": [], "error": "Bad request."}),
        400,
    )


@bp.route("", methods=["GET"])
def get_key_store():
    state = key_store_service.status()
    return jsonify(
        {
            "encryptKeyStorage": state["encrypt_key_storage"],
            "credentialRefs": state["credential_refs"],
        }
    )


@bp.route("/unlock", methods=["POST"])
def unlock_key_store():
    body = _payload()
    if body is None:
        return _bad_request()
    password = _text(body, "password")
    if password is None:
        return _bad_request()
    try:
        opened = key_store_service.unlock(password)
    except key_store_service.KeyStoreError as error:
        return _failure(error)
    return jsonify(
        {
            "successful": True,
            "credentials": [{"credentialRef": ref} for ref in sorted(opened)],
        }
    )


@bp.route("/password", methods=["POST"])
def change_key_store_password():
    body = _payload()
    if body is None:
        return _bad_request()
    old_password = _text(body, "oldPassword")
    new_password = _text(body, "newPassword")
    confirm_password = _text(body, "confirmPassword")
    if old_password is None or new_password is None or confirm_password is None:
        return _bad_request()
    try:
        key_store_service.change_password(old_password, new_password, confirm_password)
    except key_store_service.KeyStoreError as error:
        return _failure(error)
    return jsonify({"successful": True})


@bp.route("/<int:provider_id>", methods=["PUT"])
def put_provider_secret(provider_id: int):
    body = _payload()
    if body is None:
        return _bad_request()
    password = _text(body, "password")
    secret = _text(body, "secret")
    if password is None or secret is None:
        return _bad_request()
    try:
        ref = key_store_service.put_secret(provider_id, password, secret)
    except key_store_service.KeyStoreError as error:
        return _failure(error)
    return jsonify({"successful": True, "credentialRef": ref})


@bp.route("/<int:provider_id>", methods=["DELETE"])
def delete_provider_secret(provider_id: int):
    body = _payload()
    if body is None:
        return _bad_request()
    password = _text(body, "password")
    if password is None:
        return _bad_request()
    try:
        key_store_service.delete_secret(provider_id, password)
    except key_store_service.KeyStoreError as error:
        return _failure(error)
    return "", 204
