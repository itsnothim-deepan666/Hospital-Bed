import cv2
import mediapipe as mp
import math
import sys
import socket


ESP_IP="192.168.137.171"
ESP_PORT=12345

class UDP:
    def __init__(self, ip, port):
        self.sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.esp_address=(ip, port)
    def send_command(self, command):
        self.sock.sendto(command.encode(), self.esp_address)
        print(f"Command sent: {command}")
    
udpclient = UDP(ESP_IP, ESP_PORT)



# Function to calculate the Eye Aspect Ratio-like metric
def euclidean_distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def get_eye_openness(landmarks, eye_indices):
    vertical_1 = euclidean_distance(landmarks[eye_indices[1]], landmarks[eye_indices[5]])
    vertical_2 = euclidean_distance(landmarks[eye_indices[2]], landmarks[eye_indices[4]])
    horizontal = euclidean_distance(landmarks[eye_indices[0]], landmarks[eye_indices[3]])
    return (vertical_1 + vertical_2) / (2.0 * horizontal)

# Eye indices from MediaPipe's face mesh
RIGHT_EYE = [33, 160, 158, 133, 153, 144]
LEFT_EYE = [362, 385, 387, 263, 373, 380]

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1)

cap = cv2.VideoCapture(0)

single_blink_count = 0
blink_threshold = 0.2
frame_counter = 0
long=10
frames_to_confirm_blink = 3

while True:
    ret, frame = cap.read()
    if not ret:
        break
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        face_landmarks = results.multi_face_landmarks[0]
        h, w, _ = frame.shape
        landmarks = [(int(p.x * w), int(p.y * h)) for p in face_landmarks.landmark]

        left_openness = get_eye_openness(landmarks, LEFT_EYE)
        right_openness = get_eye_openness(landmarks, RIGHT_EYE)
        avg_openness = (left_openness + right_openness) / 2.0


        if avg_openness < blink_threshold:
            frame_counter += 1
        else:
            if frame_counter >= frames_to_confirm_blink:
                if frame_counter>=long:
                    command="1"
                    udpclient.send_command(command)
                    print("long blink")
                else:
                    command="0"
                    udpclient.send_command(command)
                    print("single blink")
            frame_counter = 0


        

    cv2.imshow("Blink Detection - MediaPipe", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC key
        break

cap.release()
cv2.destroyAllWindows()
