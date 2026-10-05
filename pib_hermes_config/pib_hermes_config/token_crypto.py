"""The encrypt_token / decrypt_token primitive.

The token service and the provider key store both call these functions.
Key derivation stays scrypt (n=2**14, r=8, p=1, dklen=32) and the cipher
stays Fernet, which is what token_service.py already used for the
SmartConnect token. A password shorter than 8 characters is rejected.
A wrong password raises TokenCryptoError and does not return plaintext.
"""

from __future__ import annotations

import secrets
from base64 import urlsafe_b64encode
from hashlib import scrypt
from typing import Optional

from cryptography.fernet import Fernet

MIN_PASSWORD_LENGTH = 8


class TokenCryptoError(Exception):
    """A ciphertext could not be opened. The message never contains a secret."""


def _derive_key(salt: bytes, password: str) -> bytes:
    encoded_password: bytes = str.encode(password)
    key = scrypt(encoded_password, salt=salt, n=2**14, r=8, p=1, dklen=32)
    return urlsafe_b64encode(key)


def encrypt_token(token: str, password: str) -> Optional[tuple[bytes, bytes]]:
    """Encrypt one string. Returns (salt, ciphertext), or None if refused.

    None means the password is shorter than 8 characters, or the cipher
    could not be built. Callers persist the two byte strings themselves.
    """
    if len(password) < MIN_PASSWORD_LENGTH:
        return None

    try:
        salt = secrets.token_bytes(16)
        key = _derive_key(salt, password)
        ciphertext = Fernet(key).encrypt(str.encode(token))
    except Exception:
        return None
    return salt, ciphertext


def decrypt_token(password: str, salt: bytes, ciphertext: bytes) -> str:
    """Open a ciphertext produced by encrypt_token.

    Raises TokenCryptoError when the password is wrong or the payload is
    not a Fernet token. The exception text is fixed.
    """
    try:
        key = _derive_key(salt, password)
        decrypted: bytes = Fernet(key).decrypt(ciphertext)
        return decrypted.decode("utf-8")
    except Exception:
        raise TokenCryptoError("Token could not be decrypted") from None
