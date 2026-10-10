from marshmallow import ValidationError
from sqlalchemy import event
from sqlalchemy.exc import NoResultFound, IntegrityError
from werkzeug.exceptions import MethodNotAllowed, UnprocessableEntity

from app.app import app, db
from controller import error_handler
from service import motor_service, pose_service, program_service

app.register_error_handler(ValidationError, error_handler.handle_bad_request_error)
app.register_error_handler(NoResultFound, error_handler.handle_not_found_error)
app.register_error_handler(400, error_handler.handle_bad_request_error)
app.register_error_handler(404, error_handler.handle_not_found_error)
app.register_error_handler(500, error_handler.handle_internal_server_error)
app.register_error_handler(501, error_handler.handle_not_implemented_error)
app.register_error_handler(
    UnprocessableEntity, error_handler.handle_unprocessable_entity_error
)
app.register_error_handler(
    MethodNotAllowed, error_handler.handle_method_not_allowed_error
)
app.register_error_handler(Exception, error_handler.handle_unknown_error)
app.register_error_handler(IntegrityError, error_handler.handle_bad_request_error)
app.register_error_handler(
    pose_service.PoseRefusedError, error_handler.handle_conflict_error
)
app.register_error_handler(
    pose_service.PoseValidationError, error_handler.handle_invalid_request_error
)
# A refused workspace is the caller's to fix and it is not a conflict, so it shares the
# 400-with-the-reason handler rather than getting an identical one of its own.
app.register_error_handler(
    program_service.ProgramCompilationError, error_handler.handle_invalid_request_error
)
app.register_error_handler(
    motor_service.MotorValidationError, error_handler.handle_invalid_request_error
)


def on_connect(dbapi_con, con_record):
    dbapi_con.execute("PRAGMA FOREIGN_KEYS=ON")


if __name__ == "__main__":
    with app.app_context():
        event.listen(db.engine, "connect", on_connect)
    app.run(host="0.0.0.0", port=5000)
