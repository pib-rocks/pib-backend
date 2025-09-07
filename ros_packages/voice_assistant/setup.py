import os
from glob import glob

from setuptools import find_packages, setup

package_name = "voice_assistant"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "langchain_pib"), glob("langchain_pib/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pib",
    maintainer_email="pib@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "assistant = voice_assistant.assistant:main",
            "chat = voice_assistant.chat:main",
            "audio_recorder = voice_assistant.audio_recorder:main",
            "audio_player = voice_assistant.audio_player:main",
            "token_service = voice_assistant.token_service:main",
        ],
    },
)
