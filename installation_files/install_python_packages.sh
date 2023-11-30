#!/bin/bash
#
# This script installs all python packages necessary for running pib
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Installing python packages --""$RESET_TEXT_COLOR"		

# Package installs for voice assistant
pip install openai==0.28
pip install google-cloud-texttospeech
pip install --upgrade --force-reinstall google-cloud-speech
sudo apt-get install -y portaudio19-dev
pip install pyaudio==0.2.14
pip install SpeechRecognition

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Python package installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"