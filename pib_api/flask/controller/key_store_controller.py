"""HTTP surface for the provider key store.

Responses name a wrong password and carry no secret material. The decrypted
keys stay in the key-store process. Encryption off stores cleartext and
does not ask for a password.
"""

from flask import Blueprint, jsonify, request

from service import display_prompt, key_store_service

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


def _password_for_store(body: dict) -> str | None:
    """Password from the body. Encryption off allows it to be omitted."""
    password = _text(body, "password")
    if password is not None:
        return password
    if key_store_service.encryption_enabled():
        return None
    return ""


def _opened(opened: dict):
    mode = key_store_service.operating_mode()
    if mode == key_store_service.MODE_UNLOCKED:
        display_prompt.dismiss_surface()
    return jsonify(
        {
            "successful": True,
            "mode": mode,
            "credentials": [{"credentialRef": ref} for ref in sorted(opened)],
        }
    )


@bp.route("", methods=["GET"])
def get_key_store():
    state = key_store_service.status()
    return jsonify(
        {
            "encryptKeyStorage": state["encrypt_key_storage"],
            "credentialRefs": state["credential_refs"],
            "mode": key_store_service.operating_mode(),
        }
    )


@bp.route("/unlock", methods=["POST"])
def unlock_key_store():
    body = _payload()
    if body is None:
        return _bad_request()
    password = _password_for_store(body)
    if password is None:
        return _bad_request()
    try:
        opened = key_store_service.unlock(password)
    except key_store_service.KeyStoreError as error:
        return _failure(error)
    return _opened(opened)


@bp.route("/cancel", methods=["POST"])
def cancel_key_store_prompt():
    """Dismiss the password prompt. Success, and the robot stays degraded."""
    mode = key_store_service.cancel_prompt()
    if mode == key_store_service.MODE_DEGRADED:
        display_prompt.dismiss_surface()
    return jsonify({"successful": True, "mode": mode, "credentials": []})


@bp.route("/display", methods=["GET"])
def display_password_prompt():
    """Password page the robot's own screen can open without Cerebra."""
    return display_prompt.PROMPT_PAGE, 200, {"Content-Type": "text/html; charset=utf-8"}


@bp.route("/display/unlock", methods=["POST"])
def display_unlock_key_store():
    """Same store as Cerebra's unlock, called from the on-device page."""
    return unlock_key_store()


@bp.route("/display/cancel", methods=["POST"])
def display_cancel_key_store_prompt():
    """Cancel from the on-device page. Leaves degraded mode, not an error."""
    return cancel_key_store_prompt()


@bp.route("/password", methods=["POST"])
def change_key_store_password():
    body = _payload()
    if body is None:
        return _bad_request()
    old_password = _text(body, "oldPassword") or ""
    new_password = _text(body, "newPassword")
    confirm_password = _text(body, "confirmPassword")
    # An empty old password is the "save the first one" case that the Speech tab sends
    # from its Save password button: with no store yet there is nothing to decrypt, and
    # the service still rejects a wrong old password once a store exists.
    if new_password is None or confirm_password is None:
        return _bad_request()
    try:
        key_store_service.change_password(old_password, new_password, confirm_password)
    except key_store_service.KeyStoreError as error:
        return _failure(error)
    return jsonify({"successful": True})


@bp.route("/encryption", methods=["POST"])
def set_key_store_encryption():
    """Switch encryption on or off. ``password`` is the one that direction needs."""
    body = _payload()
    if body is None:
        return _bad_request()
    enabled = body.get("enabled")
    if not isinstance(enabled, bool):
        return _bad_request()
    password = _text(body, "password") or ""
    try:
        key_store_service.set_encryption(enabled, password)
    except key_store_service.KeyStoreError as error:
        return _failure(error)
    return jsonify(
        {
            "successful": True,
            "encryptKeyStorage": key_store_service.encryption_enabled(),
            "mode": key_store_service.operating_mode(),
        }
    )


@bp.route("/<int:provider_id>", methods=["PUT"])
def put_provider_secret(provider_id: int):
    body = _payload()
    if body is None:
        return _bad_request()
    secret = _text(body, "secret")
    password = _password_for_store(body)
    if secret is None or password is None:
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
    password = _password_for_store(body)
    if password is None:
        return _bad_request()
    try:
        key_store_service.delete_secret(provider_id, password)
    except key_store_service.KeyStoreError as error:
        return _failure(error)
    return "", 204
