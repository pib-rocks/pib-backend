cmake_minimum_required(VERSION 3.8)
project(datatypes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/MotorSettings.msg"
	"msg/ChatMessage.msg"
	"msg/VoiceAssistantState.msg"
	"msg/ProgramOutputLine.msg"
	"msg/ProxyRunProgramFeedback.msg"
	"msg/ProxyRunProgramResult.msg"
	"msg/ProxyRunProgramStatus.msg"
	"msg/ProgramInput.msg"
	"msg/ChatIsListening.msg"
	"srv/ApplyMotorSettings.srv"
	"msg/ImageId.msg"
	"msg/ImageFormat.msg"
	"msg/DisplayImage.msg"
	"srv/PlayAudioFromFile.srv"
	"srv/PlayAudioFromSpeech.srv"
	"srv/GetVoiceAssistantState.srv"
	"srv/SetVoiceAssistantState.srv"
	"srv/GetChatIsListening.srv"
	"srv/SendChatMessage.srv"
	"srv/ProxyRunProgramStart.srv"
	"srv/ProxyRunProgramStop.srv"
	"srv/ClearPlaybackQueue.srv"
	"srv/GetCameraImage.srv"
	"srv/ApplyJointTrajectory.srv"
	"srv/EncryptToken.srv"
	"srv/DecryptToken.srv"
	"srv/GetTokenExists.srv"
	"action/RunProgram.action"
	"action/RecordAudio.action"
	"action/Chat.action"
	"srv/GetMicConfiguration.srv"
	DEPENDENCIES trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()