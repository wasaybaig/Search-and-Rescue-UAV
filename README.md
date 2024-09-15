# Search-and-Rescue-UAV

## Overview
This repository provides a complete end-to-end framework for a Search and Rescue (SAR) UAV. The project is designed to be highly customizable based on specific SAR scenarios. You can integrate your own fine-tuned object detection model and customize the payload for a variety of mission requirements.

## Features
- Modular framework for UAV operations
- Supports custom object detection models (e.g., YOLOv5)
- Compatible with TensorFlow Lite for onboard inference
- Payload customization based on SAR requirements
- Real-time video capture and processing
- Efficient search algorithms to optimize UAV paths

## Project Structure
- `algorithms.py`: SAR-specific search algorithms
- `copter_commands.py`: UAV command scripts
- `tflite_inference.py`: TensorFlow Lite inference for object detection
- `video_capture.py`: Real-time video feed capture and processing
- `yolov5_engine.py`: YOLOv5 model engine for object detection

## Getting Started
1. Clone the repository:
    ```bash
    git clone https://github.com/wasaybaig/Search-and-Rescue-UAV.git
    ```
2. Install the required dependencies:
    ```bash
    pip install -r requirements.txt
    ```
3. Customize the object detection model and payload settings based on your mission requirements.
4. Launch the UAV and begin search operations using the `main.py` script.

## Requirements
- Python 3.x
- TensorFlow Lite
- OpenCV
- PyMAVLink (for UAV communication)

## Demo
Check out the [Demo Video](https://drive.google.com/file/d/1Q_a6o1J586om6R37FiGkabPJGlgXHhM9/view)  and [Technical Specifications](https://drive.google.com/file/d/1YS8dZPF5gErC8Z3Ev9Y5g6d7FhhhxJ-c/view)

## License
This project is licensed under the MIT License.



