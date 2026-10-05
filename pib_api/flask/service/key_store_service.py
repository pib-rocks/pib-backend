"""Store for provider credentials.

One operator password opens every key while encryption is on. The password
and the decrypted map live in this process only. The file on disk is then
the ciphertext from encrypt_token / decrypt_token.

Encryption can be turned off. The choice is a non-secret file beside the
store, not a field inside it. Off means the store file is cleartext, there
is no operator password, and the process is unlocked without an unlock call.
A missing or unreadable settings file means encryption stays on.

The database stores an opaque credential_ref, never the secret. Nothing
here writes SOUL.md or MEMORY.md.
"""

from __future__ import annotations

import json
import logging
import os
from base64 import urlsafe_b64decode, urlsafe_b64encode
from typing import Optional

from app.app import db
from model.provider_model import Provider, RegistryModel
from pib_hermes_config.token_crypto import (
    MIN_PASSWORD_LENGTH,
    TokenCryptoError,
    decrypt_token,
    encrypt_token,
)

logger = logging.getLogger(__name__)

DEFAULT_KEY_STORE_PATH = "/home/pib/app/secrets/provider_key_store.json"
SETTINGS_FILENAME = "key_store_settings.json"
WRONG_PASSWORD_MESSAGE = "Wrong password. No keys are available."
PASSWORD_CONFIRMATION_MESSAGE = (
    "Enter the new password twice. The two entries do not match."
)
PASSWORD_TOO_SHORT_MESSAGE = "Password must be at least 8 characters."
ENCRYPTION_OFF_MESSAGE = "Encryption is off."
UNREADABLE_MESSAGE = "Key store could not be read."
UNWRITABLE_MESSAGE = "Key store could not be written."

# Named operating mode from D6. Locked, including after the prompt is
# cancelled, is degraded. Unlocked is the only other mode. Encryption off
# is unlocked: there is no password prompt at start.
MODE_DEGRADED = "degraded"
MODE_UNLOCKED = "unlocked"

# Decrypted credential_ref -> secret. None means this process is locked.
# Encryption off does not use the lock; the cleartext file is the source.
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
    """``degraded`` until a password has opened an encrypted store.

    Encryption off is ``unlocked`` with no unlock call, which is what keeps
    the start-up password prompt from opening.
    """
    if not encryption_enabled():
        return MODE_UNLOCKED
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
    """Copy of the keys this process may use.

    Encryption on returns an empty map while locked. Encryption off loads
    the cleartext file, so reading does not wait for an unlock.
    """
    if not encryption_enabled():
        if _unlocked is None and _store_exists():
            _remember(_read_cleartext())
        if _unlocked is None:
            return {}
        return dict(_unlocked)
    if _unlocked is None:
        return {}
    return dict(_unlocked)


def unlocked_secret_for_api_name(api_name: str) -> Optional[str]:
    """The unlocked secret for this catalogue id, or None.

    Locked, unknown, and a row with no secret are all None. Another
    provider's secret is not returned. The value is not logged.
    """
    if operating_mode() != MODE_UNLOCKED:
        return None
    if not isinstance(api_name, str) or api_name.strip() == "":
        return None
    secrets = unlocked_credentials()
    rows = RegistryModel.query.filter_by(api_name=api_name).all()
    for model in rows:
        provider = model.provider
        if provider is None:
            continue
        ref = provider.credential_ref
        if not ref:
            continue
        secret = secrets.get(ref)
        if isinstance(secret, str) and secret.strip():
            return secret
    return None


def store_path() -> str:
    return os.environ.get("PIB_KEY_STORE_PATH") or DEFAULT_KEY_STORE_PATH


def settings_path() -> str:
    """Non-secret mode file. ``PIB_KEY_STORE_SETTINGS_PATH`` overrides it.

    Otherwise it sits beside the store, so a ``PIB_KEY_STORE_PATH`` override
    moves both files together.
    """
    configured = os.environ.get("PIB_KEY_STORE_SETTINGS_PATH")
    if configured:
        return configured
    directory = os.path.dirname(store_path())
    return os.path.join(directory, SETTINGS_FILENAME)


def encryption_enabled() -> bool:
    """True unless the settings file explicitly turns encryption off.

    Missing, unreadable, or a file without a boolean ``encrypt_key_storage``
    stays on. That is the same behaviour as before this setting existed.
    """
    path = settings_path()
    try:
        with open(path, "r", encoding="utf-8") as handle:
            loaded = json.load(handle)
    except (OSError, UnicodeError, json.JSONDecodeError):
        return True
    if not isinstance(loaded, dict):
        return True
    enabled = loaded.get("encrypt_key_storage")
    if not isinstance(enabled, bool):
        return True
    return enabled


def credential_ref_for(provider_id: int) -> str:
    return f"provider-{provider_id}"


def status() -> dict:
    """Non-secret view. ``encrypt_key_storage`` is the persisted setting."""
    rows = (
        Provider.query.filter(Provider.credential_ref.isnot(None))
        .order_by(Provider.id)
        .all()
    )
    return {
        "encrypt_key_storage": encryption_enabled(),
        "credential_refs": [row.credential_ref for row in rows],
    }


def put_secret(provider_id: int, password: str, secret: str) -> str:
    """Store one provider secret.

    Encryption on keeps the single operator password. Encryption off ignores
    ``password``; an empty string is enough.
    """
    provider = _provider_or_raise(provider_id)
    if not isinstance(secret, str) or secret.strip() == "":
        raise KeyStoreError("A provider secret is required.", 400)

    if encryption_enabled() and not _store_exists():
        if len(password) < MIN_PASSWORD_LENGTH:
            raise KeyStoreError(PASSWORD_TOO_SHORT_MESSAGE, 400)
    mapping = _open_mapping(password)

    ref = credential_ref_for(provider.id)
    previous = provider.credential_ref
    if previous and previous != ref:
        mapping.pop(previous, None)
    mapping[ref] = secret
    _save_mapping(mapping, password)
    provider.credential_ref = ref
    db.session.flush()
    _remember(mapping)
    _pin_live_models()
    return ref


def delete_secret(provider_id: int, password: str = "") -> None:
    """Remove one secret. Personalities that point at the provider stay.

    Encryption off does not require ``password``.
    """
    provider = _provider_or_raise(provider_id)
    mapping = _open_mapping(password)

    ref = provider.credential_ref
    if ref:
        mapping.pop(ref, None)
    if _store_exists() or mapping:
        _save_mapping(mapping, password)
    provider.credential_ref = None
    db.session.flush()
    _remember(mapping)


def unlock(password: str = "") -> dict[str, str]:
    """Open every key. A wrong password raises and returns nothing.

    When no encrypted store exists yet, the result is an empty map and the
    process stays locked. That is not a wrong password. Encryption off loads
    the cleartext file and does not check ``password``.
    """
    if not encryption_enabled():
        mapping = _read_cleartext() if _store_exists() else {}
        _remember(mapping)
        _pin_live_models()
        return dict(mapping)
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
    if not encryption_enabled():
        raise KeyStoreError(ENCRYPTION_OFF_MESSAGE, 400)
    if new_password != confirm_password:
        raise KeyStoreError(PASSWORD_CONFIRMATION_MESSAGE, 400)
    mapping = _decrypt_map(old_password) if _store_exists() else {}
    _write_encrypted(mapping, new_password)
    _remember(mapping)


def set_encryption(enabled: bool, password: str = "") -> None:
    """Turn encryption on or off, rewriting the store before the setting.

    Turning off with an encrypted store requires the current password and
    rewrites the keys as cleartext. Turning on takes ``password`` as the new
    operator password and rewrites cleartext keys as ciphertext. The settings
    file is updated only after that rewrite succeeds. A failed rewrite leaves
    the previous file and the previous setting in place.
    """
    if enabled == encryption_enabled() and _store_agrees_with(enabled):
        return
    if enabled:
        _turn_encryption_on(password)
        return
    _turn_encryption_off(password)


def _turn_encryption_off(password: str) -> None:
    if not _store_exists():
        _write_settings(False)
        _remember({})
        logger.info("Key store encryption is now off")
        return
    mapping = _mapping_for_disable(password)
    previous = _read_store_bytes()
    _write_cleartext(mapping)
    try:
        _write_settings(False)
    except KeyStoreError:
        _restore_store_bytes(previous)
        raise
    _remember(mapping)
    logger.info("Key store encryption is now off")


def _turn_encryption_on(password: str) -> None:
    if len(password) < MIN_PASSWORD_LENGTH:
        raise KeyStoreError(PASSWORD_TOO_SHORT_MESSAGE, 400)
    mapping = _mapping_for_enable(password)
    previous = _read_store_bytes()
    _write_encrypted(mapping, password)
    try:
        _write_settings(True)
    except KeyStoreError:
        _restore_store_bytes(previous)
        raise
    _remember(mapping)
    logger.info("Key store encryption is now on")


def _mapping_for_disable(password: str) -> dict[str, str]:
    document = _read_document()
    if _is_cleartext_document(document):
        return _string_map(document.get("secrets"))
    if _is_envelope_document(document):
        return _decrypt_document(document, password)
    logger.warning("Key store could not be read")
    raise KeyStoreError(UNREADABLE_MESSAGE, 422)


def _mapping_for_enable(password: str) -> dict[str, str]:
    if not _store_exists():
        return {}
    document = _read_document()
    if _is_cleartext_document(document):
        return _string_map(document.get("secrets"))
    if _is_envelope_document(document):
        return _decrypt_document(document, password)
    logger.warning("Key store could not be read")
    raise KeyStoreError(UNREADABLE_MESSAGE, 422)


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


def _store_agrees_with(enabled: bool) -> bool:
    """True when the file format matches the requested mode.

    No file agrees with either mode. A cleartext file does not agree with
    encryption on, and an envelope does not agree with encryption off.
    """
    if not _store_exists():
        return True
    document = _read_document()
    if enabled:
        return _is_envelope_document(document)
    return _is_cleartext_document(document)


def _remember(mapping: dict[str, str]) -> None:
    global _unlocked
    _unlocked = dict(mapping)


def _open_mapping(password: str) -> dict[str, str]:
    """Keys already on disk. Empty when the file does not exist yet."""
    if not _store_exists():
        return {}
    if encryption_enabled():
        return _decrypt_map(password)
    return _read_cleartext()


def _save_mapping(mapping: dict[str, str], password: str) -> None:
    if encryption_enabled():
        _write_encrypted(mapping, password)
        return
    _write_cleartext(mapping)


def _decrypt_map(password: str) -> dict[str, str]:
    document = _read_document()
    if not _is_envelope_document(document):
        logger.warning("Key store could not be read")
        raise KeyStoreError(UNREADABLE_MESSAGE, 422)
    return _decrypt_document(document, password)


def _decrypt_document(document: dict, password: str) -> dict[str, str]:
    try:
        salt = urlsafe_b64decode(document["salt"].encode("ascii"))
        ciphertext = document["ciphertext"].encode("ascii")
        plaintext = decrypt_token(password, salt, ciphertext)
        loaded = json.loads(plaintext)
    except TokenCryptoError:
        logger.warning("Key store could not be decrypted")
        raise KeyStoreError(WRONG_PASSWORD_MESSAGE, 401) from None
    except (KeyError, TypeError, ValueError):
        logger.warning("Key store could not be read")
        raise KeyStoreError(UNREADABLE_MESSAGE, 422) from None
    return _string_map(loaded)


def _read_cleartext() -> dict[str, str]:
    document = _read_document()
    if not _is_cleartext_document(document):
        logger.warning("Key store could not be read")
        raise KeyStoreError(UNREADABLE_MESSAGE, 422)
    return _string_map(document.get("secrets"))


def _is_cleartext_document(document: dict) -> bool:
    """A cleartext store is marked, and it is not an envelope."""
    return (
        document.get("cleartext") is True
        and "ciphertext" not in document
        and "salt" not in document
    )


def _is_envelope_document(document: dict) -> bool:
    """An envelope has salt and ciphertext, and it is not marked cleartext."""
    return (
        document.get("cleartext") is not True
        and isinstance(document.get("salt"), str)
        and isinstance(document.get("ciphertext"), str)
    )


def _string_map(loaded: object) -> dict[str, str]:
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
    envelope = {
        "version": 1,
        "encrypt_key_storage": True,
        "salt": urlsafe_b64encode(salt).decode("ascii"),
        "ciphertext": ciphertext.decode("ascii"),
    }
    data = json.dumps(envelope, separators=(",", ":"), sort_keys=True).encode("utf-8")
    _atomic_write(store_path(), data)


def _write_cleartext(mapping: dict[str, str]) -> None:
    document = {
        "version": 1,
        "cleartext": True,
        "secrets": mapping,
    }
    data = json.dumps(document, separators=(",", ":"), sort_keys=True).encode("utf-8")
    _atomic_write(store_path(), data)


def _write_settings(enabled: bool) -> None:
    document = {"encrypt_key_storage": bool(enabled)}
    data = json.dumps(document, separators=(",", ":"), sort_keys=True).encode("utf-8")
    _atomic_write(settings_path(), data)


def _read_document() -> dict:
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


def _read_store_bytes() -> Optional[bytes]:
    path = store_path()
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "rb") as handle:
            return handle.read()
    except OSError:
        logger.warning("Key store could not be read")
        raise KeyStoreError(UNREADABLE_MESSAGE, 422) from None


def _restore_store_bytes(previous: Optional[bytes]) -> None:
    """Put the previous store file back after a settings write failed."""
    path = store_path()
    if previous is None:
        try:
            os.unlink(path)
        except OSError:
            logger.error("Key store could not be restored")
        return
    try:
        _atomic_write(path, previous)
    except KeyStoreError:
        logger.error("Key store could not be restored")


def _atomic_write(path: str, data: bytes) -> None:
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, mode=0o700, exist_ok=True)
        try:
            os.chmod(directory, 0o700)
        except OSError:
            pass

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
    try:
        os.replace(temporary, path)
    except OSError:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        logger.error("Key store could not be written")
        raise KeyStoreError(UNWRITABLE_MESSAGE, 500) from None
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass
