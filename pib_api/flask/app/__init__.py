
from app.app import app
from controller import assistant_controller
from route import program_route, chat_route, motor_route, personality_route, camera_route, bricklet_route


app.register_blueprint(program_route.blueprint, url_prefix='/program', name='program')
app.register_blueprint(chat_route.blueprint, url_prefix='/voice-assistant/chat', name='chat')
app.register_blueprint(motor_route.blueprint, url_prefix='/motor', name='motor')
app.register_blueprint(personality_route.blueprint, url_prefix='/voice-assistant/personality', name='personality')
app.register_blueprint(camera_route.blueprint, url_prefix='/camera-settings', name='camera')
app.register_blueprint(bricklet_route.blueprint, url_prefix='/bricklet', name='bricklet')
app.register_blueprint(assistant_controller.bp, url_prefix='/assistant-model', name='assistant_model')
