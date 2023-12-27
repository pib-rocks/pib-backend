from app.app import app, db
from route import program_route
from route import program_route, chat_route, motor_settings_route, personality_route, camera_route
from controller import error_handler;
from sqlalchemy import event

app.register_blueprint(program_route.blueprint, url_prefix='/program', name='program')
app.register_blueprint(chat_route.blueprint, url_prefix='/voice-assistant/chat', name='chat')
app.register_blueprint(motor_settings_route.blueprint, url_prefix='/motor-settings', name='motor_settings')
app.register_blueprint(personality_route.blueprint, url_prefix='/voice-assistant/personality', name='personality')
app.register_blueprint(camera_route.blueprint, url_prefix='/camera-settings', name='camera')

app.register_error_handler(404, error_handler.handle_not_found_error)
app.register_error_handler(500, error_handler.handle_internal_server_error)

def on_connect(dbapi_con, con_record):
    dbapi_con.execute('PRAGMA FOREIGN_KEYS=ON')

if __name__ == '__main__':
    with app.app_context():
        event.listen(db.engine, 'connect', on_connect)
    app.run(host="0.0.0.0", port=5000)

