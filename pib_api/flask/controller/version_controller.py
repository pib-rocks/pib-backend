from flask import Blueprint, jsonify

from service.revision_service import installed_revisions
from service.version_service import read_app_version

bp = Blueprint("version_controller", __name__)


@bp.route("", methods=["GET"])
def get_version():
    """Image version plus the checked-out revisions of the repositories.

    The revision values come from the host update runner; anything it has not
    recorded yet is reported as ``unknown`` instead of being guessed.
    """
    revisions = installed_revisions()
    return (
        jsonify(
            {
                "version": read_app_version(),
                "repositories": revisions["repositories"],
            }
        ),
        200,
    )
