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
    marimo_controller,
    microphone_array_controller,
    system_controller,
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
app.register_blueprint(marimo_controller.marimo_bp, url_prefix="/marimo", name="marimo")
app.register_blueprint(marimo_controller.marimo_bp, url_prefix="/v1/marimo", name="marimo_v1")
app.register_blueprint(marimo_controller.marimo_bp, url_prefix="/api/v1/marimo", name="marimo_api_v1")
app.register_blueprint(
    microphone_array_controller.bp,
    url_prefix="/system/microphone-array",
    name="microphone_array",
)
app.register_blueprint(
    microphone_array_controller.bp,
    url_prefix="/v1/system/microphone-array",
    name="microphone_array_v1",
)
app.register_blueprint(
    system_controller.bp,
    url_prefix="/system",
    name="system",
)
app.register_blueprint(
    system_controller.bp,
    url_prefix="/v1/system",
    name="system_v1",
)
app.register_blueprint(
    system_controller.bp,
    url_prefix="/api/system",
    name="system_api",
)
app.register_blueprint(
    system_controller.bp,
    url_prefix="/api/v1/system",
    name="system_api_v1",
)
