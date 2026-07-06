⚠️ This repository is a custom fork of the original pib-rocks project and contains significant modifications not available upstream.

pib-backend (Joshi-1337 Fork)

This repository is a custom fork of the original pib-rocks backend project.

Original project:

* https://github.com/pib-rocks/pib-backend

This fork contains custom ROS services, Blockly extensions, robot control improvements and deployment changes.

## Fork Information

The repository has diverged from the upstream project and includes custom functionality not available in the original pib-rocks implementation.

The original `pib-blockly` submodule has been integrated directly into this repository.

## Added Features

### TinkerForge RGB Button Service

Added a new ROS service package:

ros_packages/button_service

Features:

* Read button state
* Wait for button press
* Read switch state
* Control RGB button colors

### Blockly Button Blocks

Added Blockly support for:

* Pushbutton block
* Switch block
* Set button color block

including generator and backend integration.

### Audio Playback Support

Added Blockly support for sound file playback.

Features:

* Generic sound playback
* 20 predefined sound files
* R2D2 sound support

### Face Tracking Fix

Reworked face tracking implementation.

Changes:

* Switched to ROS camera service coordinate output
* Improved Blockly generator compatibility
* Reliable Python runtime behaviour

### Implemented Object recognition
* detect object as specified
* count specified object
* detect difference between specified objects
* image description 

### Implemented "Facial" Expressions
* Display 10 expressions like angry, sad or happy

### Program and Pose Transfer Support

Added backend support required for:

* Program import/export
* Pose import/export

### Deployment Changes

Installation and update scripts have been modified to use the repositories maintained under:

* https://github.com/Joshi-1337/cerebra
* https://github.com/Joshi-1337/pib-backend

instead of the original pib-rocks repositories.

## Repository Structure Changes

The former Git submodule:

pib_blockly/pib_blockly_server/src/pib-blockly

has been converted into regular source files and is maintained directly within this repository.

## Compatibility

This fork remains based on the pib-rocks architecture but may not be fully compatible with future upstream releases without manual integration work.
