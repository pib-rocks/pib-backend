"""Read the backend version embedded in the container image."""

VERSION_FILES = (
    "/etc/pib_version",
    "/app/version.py",
)  # /etc survives the /app volume mount

REQUIRED_PIB_SDK_VERSION_FILES = ("/etc/pib_required_pib_sdk_version",)


def read_app_version() -> str:
    for path in VERSION_FILES:
        try:
            with open(path, encoding="utf-8") as version_file:
                value = version_file.read().strip().strip('"').strip("'")
            if value:
                return value
        except (OSError, UnicodeError):
            continue
    return "unknown"


def read_required_pib_sdk_version() -> str:
    for path in REQUIRED_PIB_SDK_VERSION_FILES:
        try:
            with open(path, encoding="utf-8") as version_file:
                value = version_file.read().strip()
            if value:
                return value
        except (OSError, UnicodeError):
            continue
    return "unknown"
