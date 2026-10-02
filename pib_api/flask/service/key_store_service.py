"""Encrypted store for provider credentials.

One operator password opens every key. The password and the decrypted map
live in this process only. The file on disk is the ciphertext from
encrypt_token / decrypt_token. The database stores an opaque credential_ref,
never the secret. Nothing here writes SOUL.md or MEMORY.md.
"""

from __future__ import annotations

import json
import logging
import os
from base64 import urlsafe_b64decode, urlsafe_b64encode
from typing import Optional

from app.app import db
from model.provider_model import Provider
from pib_hermes_config.token_crypto import (
    MIN_PASSWORD_LENGTH,
    TokenCryptoError,
    decrypt_token,
    encrypt_token,
)

logger = logging.getLogger(__name__)

DEFAULT_KEY_STORE_PATH = "/home/pib/app/secrets/provider_key_store.json"
WRONG_PASSWORD_MESSAGE = "Wrong password. No keys are available."
PASSWORD_CONFIRMATION_MESSAGE = (
    "Enter the new password twice. The two entries do not match."
)
PASSWORD_TOO_SHORT_MESSAGE = "Password must be at least 8 characters."
UNREADABLE_MESSAGE = "Key store could not be read."
UNWRITABLE_MESSAGE = "Key store could not be written."

# Named operating mode from D6. Locked, including after the prompt is
# cancelled, is degraded. Unlocked is the only other mode.
MODE_DEGRADED = "degraded"
MODE_UNLOCKED = "unlocked"

# Decrypted credential_ref -> secret. None means this process is locked.
_unlocked: Optional[dict[str, str]] = None


class KeyStoreError(Exception):
    """A key-store operation stopped. message is safe to show in the UI."""

    def __init__(self, message: str, status_code: int) -> None:
        super().__init__(message)
        self.status_code = status_code


def lock() -> None:
    """Drop decrypted keys. The password is not kept."""
    global _unlocked
    _unlocked = None


def operating_mode() -> str:
    """``degraded`` until a password has opened the store in this process."""
    if _unlocked is None:
        return MODE_DEGRADED
    return MODE_UNLOCKED


def cancel_prompt() -> str:
    """The operator dismissed the password prompt.

    This is not a failed unlock. The store stays as it is: locked means the
    named degraded mode, and a store that is already open stays open.
    """
    mode = operating_mode()
    logger.info("Password prompt cancelled; operating mode is %s", mode)
    return mode


def unlocked_credentials() -> dict[str, str]:
    """Copy of the keys held in this process, or empty when locked."""
    if _unlocked is None:
        return {}
    return dict(_unlocked)


def store_path() -> str:
    return os.environ.get("PIB_KEY_STORE_PATH") or DEFAULT_KEY_STORE_PATH


def credential_ref_for(provider_id: int) -> str:
    return f"provider-{provider_id}"


def status() -> dict:
    """Non-secret view. Encryption is always on."""
    rows = (
        Provider.query.filter(Provider.credential_ref.isnot(None))
        .order_by(Provider.id)
        .all()
    )
    return {
        "encrypt_key_storage": True,
        "credential_refs": [row.credential_ref for row in rows],
    }


def put_secret(provider_id: int, password: str, secret: str) -> str:
    """Store one provider secret under the single operator password."""
    provider = _provider_or_raise(provider_id)
    if not isinstance(secret, str) or secret.strip() == "":
        raise KeyStoreError("A provider secret is required.", 400)

    if _store_exists():
        mapping = _decrypt_map(password)
    else:
        if len(password) < MIN_PASSWORD_LENGTH:
            raise KeyStoreError(PASSWORD_TOO_SHORT_MESSAGE, 400)
        mapping = {}

    ref = credential_ref_for(provider.id)
    previous = provider.credential_ref
    if previous and previous != ref:
        mapping.pop(previous, None)
    mapping[ref] = secret
    _write_encrypted(mapping, password)
    provider.credential_ref = ref
    db.session.flush()
    _remember(mapping)
    _pin_live_models()
    return ref


def delete_secret(provider_id: int, password: str) -> None:
    """Remove one secret. Personalities that point at the provider stay."""
    provider = _provider_or_raise(provider_id)
    if _store_exists():
        mapping = _decrypt_map(password)
    else:
        mapping = {}

    ref = provider.credential_ref
    if ref:
        mapping.pop(ref, None)
    if _store_exists() or mapping:
        _write_encrypted(mapping, password)
    provider.credential_ref = None
    db.session.flush()
    _remember(mapping)


def unlock(password: str) -> dict[str, str]:
    """Open every key. A wrong password raises and returns nothing.

    When no store exists yet, the result is an empty map and the process
    stays locked. That is not a wrong password.
    """
    if not _store_exists():
        return {}
    mapping = _decrypt_map(password)
    _remember(mapping)
    _pin_live_models()
    return dict(mapping)


def change_password(
    old_password: str, new_password: str, confirm_password: str
) -> None:
    """Re-encrypt the whole store. The first call creates it.

    A fresh robot has no store file, and the store only comes into being by setting a
    password, so demanding the file here made the first password impossible to set.
    Without a store there is nothing to decrypt and the old password is not required.
    """
    if new_password != confirm_password:
        raise KeyStoreError(PASSWORD_CONFIRMATION_MESSAGE, 400)
    mapping = _decrypt_map(old_password) if _store_exists() else {}
    _write_encrypted(mapping, new_password)
    _remember(mapping)


def _pin_live_models() -> None:
    """Best-effort. A model-list failure must not lock the keys back up."""
    try:
        from service import live_model_service

        live_model_service.pin_unlocked_providers()
    except Exception:
        logger.warning(
            "Could not pin live models from the account model list.",
            exc_info=True,
        )


def _provider_or_raise(provider_id: int) -> Provider:
    provider = Provider.query.filter_by(id=provider_id).first()
    if provider is None:
        raise KeyStoreError("Provider not found.", 404)
    return provider


def _store_exists() -> bool:
    return os.path.isfile(store_path())


def _remember(mapping: dict[str, str]) -> None:
    global _unlocked
    _unlocked = dict(mapping)


def _decrypt_map(password: str) -> dict[str, str]:
    envelope = _read_envelope()
    try:
        salt = urlsafe_b64decode(envelope["salt"].encode("ascii"))
        ciphertext = envelope["ciphertext"].encode("ascii")
        plaintext = decrypt_token(password, salt, ciphertext)
        loaded = json.loads(plaintext)
    except TokenCryptoError:
        logger.warning("Key store could not be decrypted")
        raise KeyStoreError(WRONG_PASSWORD_MESSAGE, 401) from None
    except (KeyError, TypeError, ValueError):
        logger.warning("Key store could not be read")
        raise KeyStoreError(UNREADABLE_MESSAGE, 422) from None

    if not isinstance(loaded, dict) or any(
        not isinstance(value, str) for value in loaded.values()
    ):
        logger.warning("Key store could not be read")
        raise KeyStoreError(UNREADABLE_MESSAGE, 422)
    return {str(key): value for key, value in loaded.items()}


def _write_encrypted(mapping: dict[str, str], password: str) -> None:
    payload = json.dumps(mapping, separators=(",", ":"), sort_keys=True)
    encrypted = encrypt_token(payload, password)
    if encrypted is None:
        if len(password) < MIN_PASSWORD_LENGTH:
            raise KeyStoreError(PASSWORD_TOO_SHORT_MESSAGE, 400)
        logger.error("Key store could not be encrypted")
        raise KeyStoreError(UNWRITABLE_MESSAGE, 500)
    salt, ciphertext = encrypted
    _write_envelope(salt, ciphertext)


def _read_envelope() -> dict:
    path = store_path()
    try:
        with open(path, "r", encoding="utf-8") as handle:
            loaded = json.load(handle)
    except (OSError, json.JSONDecodeError):
        logger.warning("Key store could not be read")
        raise KeyStoreError(UNREADABLE_MESSAGE, 422) from None
    if not isinstance(loaded, dict):
        logger.warning("Key store could not be read")
        raise KeyStoreError(UNREADABLE_MESSAGE, 422)
    return loaded


def _write_envelope(salt: bytes, ciphertext: bytes) -> None:
    path = store_path()
    directory = os.path.dirname(path)
    os.makedirs(directory, mode=0o700, exist_ok=True)
    try:
        os.chmod(directory, 0o700)
    except OSError:
        pass

    envelope = {
        "version": 1,
        "encrypt_key_storage": True,
        "salt": urlsafe_b64encode(salt).decode("ascii"),
        "ciphertext": ciphertext.decode("ascii"),
    }
    data = json.dumps(envelope, separators=(",", ":"), sort_keys=True).encode("utf-8")
    temporary = f"{path}.tmp"
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        logger.error("Key store could not be written")
        raise KeyStoreError(UNWRITABLE_MESSAGE, 500) from None
    os.chmod(temporary, 0o600)
    os.replace(temporary, path)
    os.chmod(path, 0o600)
