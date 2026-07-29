from app.app import app
from controller import (
    assistant_model_controller,
    bricklet_controller,
    camera_controller,
    chat_controller,
    diagnostics_controller,
    motor_controller,
    personality_controller,
    program_controller,
    pose_controller,
    ip_controller,
    button_program_controller,
    docker_controller,
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
app.register_blueprint(pose_controller.bp, url_prefix="/pose", name="pose")
app.register_blueprint(ip_controller.bp, url_prefix="/host-ip", name="host-ip")
app.register_blueprint(
    button_program_controller.bp, url_prefix="/button-programs", name="button-programs"
)
app.register_blueprint(
    diagnostics_controller.bp, url_prefix="/v1/diagnostics", name="diagnostics_v1"
)
app.register_blueprint(
    diagnostics_controller.bp, url_prefix="/diagnostics", name="diagnostics"
)
app.register_blueprint(
    diagnostics_controller.bp, url_prefix="/api/v1/diagnostics", name="diagnostics_api_v1"
)
app.register_blueprint(docker_controller.bp, url_prefix="/docker", name="docker")
app.register_blueprint(docker_controller.bp, url_prefix="/v1/docker", name="docker_v1")
app.register_blueprint(docker_controller.bp, url_prefix="/api/v1/docker", name="docker_api_v1")
