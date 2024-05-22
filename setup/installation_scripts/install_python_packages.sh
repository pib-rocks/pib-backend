#!/bin/bash
#
# This script installs all python packages necessary for running pib
# To properly run this script relies on being sourced by the "setup-pib.sh"-script

echo -e "$YELLOW_TEXT_COLOR""-- Installing python packages --""$RESET_TEXT_COLOR"		

# Package installs for voice assistant
pip install openai==1.11.0
pip install google-cloud-texttospeech
pip install --upgrade --force-reinstall google-cloud-speech
sudo apt-get install -y portaudio19-dev
pip install pyaudio==0.2.14
pip install SpeechRecognition
pip install boto3

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Python package installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"

# Python code variables
PYTHON_CODE_PATH="$USER_HOME/cerebra_programs"
INIT_PYTHON_CODE="print('hello world')"

# Create the directory for python code and populate it with a single initial python script (matching
# the single entry in the database)
mkdir "$PYTHON_CODE_PATH"
echo "$INIT_PYTHON_CODE" | cat > "$PYTHON_CODE_PATH/e1d46e2a-935e-4e2b-b2f9-0856af4257c5.py"