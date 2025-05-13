import os
import pickle
import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2
import paho.mqtt.client as mqtt
import json
import time
import socket
import threading

# MQTT Configuration
MQTT_BROKER = "192.168.137.241"
MQTT_PORT = 1883
MQTT_TOPIC_RELAY1 = "home/relay1"
MQTT_TOPIC_RELAY2 = "home/relay2"

# TCP Configuration for backup
TCP_HOST = "192.168.137.241"
TCP_PORT = 9999

# Initialize TCP socket
tcp_socket = None
try:
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((TCP_HOST, TCP_PORT))
    print("Connected to TCP server")
except Exception as e:
    print(f"Failed to connect to TCP server: {e}")

# MQTT Callbacks
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code: {rc}")

def on_publish(client, userdata, mid):
    print(f"Message Published: {mid}")

# Initialize MQTT Client
client = mqtt.Client()
client.on_connect = on_connect
client.on_publish = on_publish

mqtt_connected = False
try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
    mqtt_connected = True
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")
    print("Will try using TCP communication")

# Load the trained model
model_dict = pickle.load(open('./model.p', 'rb'))
model = model_dict['model']

def init_camera():
    try:
        # Initialize the Raspberry Pi Camera using Picamera2
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
        picam2.start()
        time.sleep(2)  # Add delay to ensure camera is fully initialized
        print("Camera initialized successfully")
        return picam2
    except Exception as e:
        print(f"Failed to initialize camera: {e}")
        return None

# Initialize camera
picam2 = init_camera()
if picam2 is None:
    if tcp_socket:
        tcp_socket.close()
    client.loop_stop()
    client.disconnect()
    exit(1)

# Initialize MediaPipe Hands and Drawing
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
hands = mp_hands.Hands(static_image_mode=False, min_detection_confidence=0.3)

# Define labels for predictions and their corresponding relay actions
labels_dict = {0: 'A', 1: 'B', 2: 'C', 3: 'D'}

# Relay control mapping
relay_actions = {
    'A': {'relay1': 'ON', 'relay2': 'OFF'},   # Turn on relay 1, turn off relay 2
    'B': {'relay1': 'OFF', 'relay2': 'ON'},   # Turn off relay 1, turn on relay 2
    'C': {'relay1': 'ON', 'relay2': 'ON'},    # Turn on both relays
    'D': {'relay1': 'OFF', 'relay2': 'OFF'}   # Turn off both relays
}

last_gesture = None
gesture_cooldown = 1  # Cooldown in seconds
last_gesture_time = 0

print("Starting gesture recognition... Press Ctrl+C to exit")

# Set up display
os.environ['DISPLAY'] = ':0'
cv2.namedWindow("Hand Gesture Recognition", cv2.WINDOW_NORMAL)
cv2.moveWindow("Hand Gesture Recognition", 0, 0)
cv2.setWindowProperty("Hand Gesture Recognition", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

try:
    while True:
        data_aux = []
        x_ = []
        y_ = []

        frame = picam2.capture_array()  # Comes as RGB from Picamera2
        results = hands.process(frame)  # MediaPipe expects RGB

        if results.multi_hand_landmarks:
            # Draw hand landmarks
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )
            hand_landmarks = results.multi_hand_landmarks[0]
            for i in range(len(hand_landmarks.landmark)):
                x = hand_landmarks.landmark[i].x
                y = hand_landmarks.landmark[i].y
                x_.append(x)
                y_.append(y)

            for i in range(len(hand_landmarks.landmark)):
                x = hand_landmarks.landmark[i].x
                y = hand_landmarks.landmark[i].y
                data_aux.append(x - min(x_))
                data_aux.append(y - min(y_))

            if len(data_aux) == 42:
                prediction = model.predict([np.asarray(data_aux)])
                predicted_character = labels_dict[int(prediction[0])]
                
                # Display gesture label
                cv2.putText(frame, f"Gesture: {predicted_character}", (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                print(f"Detected gesture: {predicted_character}")

                # Check cooldown before sending new gesture
                current_time = time.time()
                if (current_time - last_gesture_time) >= gesture_cooldown:
                    if predicted_character != last_gesture:
                        # Send MQTT message
                        relay_state = relay_actions[predicted_character]
                        try:
                            # Try MQTT first, fallback to TCP
                            if mqtt_connected:
                                try:
                                    client.publish(MQTT_TOPIC_RELAY1, relay_state['relay1'])
                                    client.publish(MQTT_TOPIC_RELAY2, relay_state['relay2'])
                                    print(f"Published via MQTT: Relay1={relay_state['relay1']}, Relay2={relay_state['relay2']}")
                                except Exception as e:
                                    print(f"MQTT publish failed: {e}")
                                    mqtt_connected = False
                            
                            # Send via TCP if MQTT failed or not connected
                            if not mqtt_connected and tcp_socket:
                                try:
                                    # Send gesture number directly (0-3)
                                    tcp_socket.sendall(str(prediction[0]).encode())
                                    print(f"Sent via TCP: Gesture {prediction[0]}")
                                except Exception as e:
                                    print(f"TCP send failed: {e}")
                            last_gesture = predicted_character
                            last_gesture_time = current_time
                        except Exception as e:
                            print(f"Failed to publish MQTT message: {e}")

        # Convert to BGR for OpenCV display
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Show the frame
        cv2.imshow("Hand Gesture Recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except (KeyboardInterrupt, KeyError):
    print("\nStopping gesture recognition...")
finally:
    cv2.destroyAllWindows()
    if picam2:
        picam2.stop()
    if mqtt_connected:
        client.loop_stop()
        client.disconnect()
    if tcp_socket:
        tcp_socket.close()
    print("Cleanup complete")
