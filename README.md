# Sign Language Controlled IoT Relay System with Raspberry Pi and ESP32

A sophisticated IoT system that uses hand gesture recognition to control electrical devices through relays. The system leverages computer vision, machine learning, and MQTT communication to create an intuitive and accessible way to control home appliances using sign language gestures.

## System Architecture

The system consists of three main components:

1. **Gesture Recognition System (Raspberry Pi)**
   - Captures hand gestures using PiCamera
   - Processes images using MediaPipe for hand landmark detection
   - Classifies gestures using a trained Random Forest model
   - Publishes commands via MQTT/TCP to control relays

2. **Relay Control System (ESP32)**
   - Controls two relays based on received MQTT commands
   - Provides visual feedback through LEDs
   - Reports relay states back to the system

3. **Node-RED Dashboard**
   - Provides a web interface for monitoring and manual control
   - Displays current gesture status
   - Shows real-time relay states
   - Allows manual override through UI switches

## Features

- **4 Gesture Controls:**
  - Gesture A: Turn ON Relay 1, Turn OFF Relay 2
  - Gesture B: Turn OFF Relay 1, Turn ON Relay 2
  - Gesture C: Turn ON both relays
  - Gesture D: Turn OFF both relays

- **Dual Communication Protocols:**
  - Primary: MQTT for reliable messaging
  - Backup: TCP socket connection for failover

- **Real-time Visual Feedback:**
  - On-screen gesture recognition display
  - LED indicators on ESP32
  - Web-based dashboard interface

## Installation

### Prerequisites

- Raspberry Pi with camera module
- ESP32 development board
- 2-channel relay module
- Python 3.7+
- Node-RED
- MQTT Broker (e.g., Mosquitto)

### Setup Instructions

1. **Raspberry Pi Setup**
   ```bash
   # Clone the repository
   git clone [repository-url]
   cd Sign-Language-Controlled-IoT-Relay-System-with-Raspberry-Pi-and-ESP32

   # Install required Python packages
   pip install -r requirements.txt
   ```

2. **ESP32 Setup**
   - Open `esp32_relay_control/esp32_relay_control.ino` in Arduino IDE
   - Update WiFi credentials and MQTT broker settings
   - Flash the code to ESP32

3. **Node-RED Setup**
   - Import `nodered_flow.json` into Node-RED
   - Configure MQTT broker settings
   - Deploy the flow

## Usage

### Training the Gesture Recognition Model

1. **Collect Training Data**
   ```bash
   python collect_imgs.py
   ```
   - Follow on-screen instructions to capture gesture images
   - Press 'q' to start capturing each gesture set
   - The system will collect 100 images per gesture

2. **Create Dataset**
   ```bash
   python create_dataset.py
   ```
   - Processes collected images using MediaPipe
   - Extracts hand landmarks
   - Creates training dataset

3. **Train Model**
   ```bash
   python train_classifier.py
   ```
   - Trains Random Forest classifier
   - Saves model to `model.p`

### Running the System

1. **Start the Control System**
   ```bash
   python gesture_control_mqtt.py
   ```
   - Initializes camera and gesture recognition
   - Establishes MQTT/TCP connections
   - Begins monitoring for gestures

2. **Access Dashboard**
   - Open Node-RED dashboard in web browser
   - Monitor gesture recognition status
   - Control relays manually if needed

## Hardware Connections

### ESP32 Pinout
- Relay 1: GPIO26
- Relay 2: GPIO27
- LED 1: GPIO2 (Built-in)
- LED 2: GPIO4

## Troubleshooting

- If MQTT connection fails, the system automatically switches to TCP
- Check LED indicators on ESP32 for connection status
- Monitor Node-RED debug output for communication issues
- Verify MQTT broker is running and accessible

## Contributing

Contributions are welcome! Please feel free to submit pull requests.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
