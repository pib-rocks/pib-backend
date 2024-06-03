from app.app import app
from controller import (
    assistant_model_controller,
    bricklet_controller,
    camera_controller,
    chat_controller,
    motor_controller,
    personality_controller,
    program_controller,
)

app.register_blueprint(program_controller.bp, url_prefix="/program", name="program")
app.register_blueprint(
    chat_controller.bp, url_prefix="/voice-assistant/chat", name="chat"
)
app.register_blueprint(motor_controller.bp, url_prefix="/motor", name="motor")
app.register_blueprint(
    personality_controller.bp,
    url_prefix="/voice-assistant/personality",
    name="personality",
)
app.register_blueprint(
    camera_controller.bp, url_prefix="/camera-settings", name="camera"
)
app.register_blueprint(bricklet_controller.bp, url_prefix="/bricklet", name="bricklet")
app.register_blueprint(
    assistant_model_controller.bp, url_prefix="/assistant-model", name="assistant_model"
)
