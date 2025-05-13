import pickle
import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2

# Load the trained model
model_dict = pickle.load(open('./model.p', 'rb'))
model = model_dict['model']

# Initialize the Raspberry Pi Camera using Picamera2
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))# 640x480
picam2.start()

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

hands = mp_hands.Hands(static_image_mode=False, min_detection_confidence=0.3)

# Define labels for predictions
labels_dict = {0: 'A', 1: 'B', 2: 'C', 3: 'D'} # Adjust this according to your dataset

while True:
    data_aux = []
    x_ = []
    y_ = []

    # Capture a frame from the Raspberry Pi Camera
    frame = picam2.capture_array()

    H, W, _ = frame.shape

    # Convert the frame to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame with MediaPipe Hands
    results = hands.process(frame_rgb)
    if results.multi_hand_landmarks:
        # Process only the first detected hand
        hand_landmarks = results.multi_hand_landmarks[0]
        mp_drawing.draw_landmarks(
            frame,  # image to draw
            hand_landmarks,  # model output
            mp_hands.HAND_CONNECTIONS,  # hand connections
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style()
        )

        # Extract hand landmark coordinates
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

        # Calculate bounding box for the hand
        x1 = int(min(x_) * W) - 10
        y1 = int(min(y_) * H) - 10
        x2 = int(max(x_) * W) - 10
        y2 = int(max(y_) * H) - 10

        # Ensure the number of features matches the model's input
        if len(data_aux) == 42:  # 21 landmarks * 2 (x, y) = 42 features
            prediction = model.predict([np.asarray(data_aux)])
            predicted_character = labels_dict[int(prediction[0])]

            # Draw the bounding box and predicted label on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 4)
            cv2.putText(frame, predicted_character, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 3,
                        cv2.LINE_AA)
        else:
            print("Warning: Incorrect number of features. Skipping prediction.")

    # Display the frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
        break

# Stop the camera and close all OpenCV windows
picam2.stop()
cv2.destroyAllWindows()