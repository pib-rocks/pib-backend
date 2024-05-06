from marshmallow import ValidationError
from sqlalchemy import event
from sqlalchemy.exc import NoResultFound

from app.app import app, db
from controller import error_handler

app.register_error_handler(ValidationError, error_handler.handle_bad_request_error)
app.register_error_handler(NoResultFound, error_handler.handle_not_found_error)
app.register_error_handler(400, error_handler.handle_bad_request_error)
app.register_error_handler(404, error_handler.handle_not_found_error)
app.register_error_handler(500, error_handler.handle_internal_server_error)
app.register_error_handler(501, error_handler.handle_not_implemented_error)
app.register_error_handler(Exception, error_handler.handle_unknown_error)


def on_connect(dbapi_con, con_record):
    dbapi_con.execute('PRAGMA FOREIGN_KEYS=ON')


if __name__ == '__main__':
    with app.app_context():
        event.listen(db.engine, 'connect', on_connect)
    app.run(host="0.0.0.0", port=5000)