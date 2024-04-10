cmake_minimum_required(VERSION 3.8)
project(datatypes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/MotorSettings.msg"
	"msg/ChatMessage.msg"
	"msg/VoiceAssistantState.msg"
	"msg/ProgramOutputLine.msg"
	"msg/ProxyRunProgramFeedback.msg"
	"msg/ProxyRunProgramResult.msg"
	"msg/ProxyRunProgramStatus.msg"
	"srv/MotorSettingsSrv.srv"
	"srv/GetVoiceAssistantState.srv"
	"srv/SetVoiceAssistantState.srv"
	"srv/ProxyRunProgramStart.srv"
	"srv/ProxyRunProgramStop.srv"
	"srv/TextToSpeechPush.srv"
	"srv/TextToSpeechClear.srv"
    "srv/GetCameraImageSrv.srv"
	"action/RunProgram.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
