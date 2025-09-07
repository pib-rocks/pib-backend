"""
Langchain Client Package
------------------------

Bündelt Proxy, Service und Tryb-Integration in einem Paket.
"""

from .langchain_proxy import app as proxy_app   # falls FastAPI-App nach außen gezeigt werden soll
from .langchain_service import LangchainService
from .langchain_tryb import TrybClient, TrybModel

__all__ = [
    "proxy_app",
    "LangchainService",
    "TrybClient",
    "TrybModel",
]
