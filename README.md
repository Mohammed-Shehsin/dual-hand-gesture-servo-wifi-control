# Dual-Hand Gesture Servo Control over Wi-Fi (ESP32 + OpenCV + MediaPipe)

A real-time computer vision project for controlling two MG90S servos independently using hand gestures. 
The system uses MediaPipe Hand Tracking to measure pinch distance between the thumb and index finger of each hand, 
mapping it to servo angles (0–180°). The angles are sent via UDP over Wi-Fi to an ESP32, which drives the servos.

---

## 🔧 Features
- Dual-hand gesture tracking (Left → Servo2, Right → Servo1)
- Real-time camera visualization with calibration
- Wi-Fi Station mode (connects to existing router or hotspot)
- UDP-based low-latency communication (no serial cable required)
- Both-hands-zero latch for reset control
- Adjustable window size and camera index support
- Calibrate gesture range using keys:  
  - `Z` / `X` for Left hand min/max  
  - `N` / `M` for Right hand min/max

---

## 🧠 System Architecture
Camera → Python (OpenCV + MediaPipe) → Wi-Fi UDP → ESP32 → MG90S Servos

---

## ⚙️ Requirements

### On PC
- Python 3.8+
- Libraries:
  ```bash
  pip install opencv-python mediapipe numpy
  ```
- A webcam
- Same Wi-Fi network as the ESP32

### On ESP32
- Arduino IDE with ESP32 core installed
- MG90S or SG90 servos on pins **GPIO18** and **GPIO19**
- Common GND with external 5V supply (≥2A recommended)

---

## 🚀 Setup Steps

### 1️⃣ Flash the ESP32
1. Open `dual_servo_udp_sta.ino` in Arduino IDE.
2. Edit:
   ```cpp
   const char* WIFI_SSID = "YourNetwork";
   const char* WIFI_PASS = "YourPassword";
   ```
3. Upload to ESP32.
4. Open Serial Monitor @115200 → note the printed **IP address**.

---

### 2️⃣ Run Python Controller
1. Connect your laptop to the **same Wi-Fi network**.
2. In terminal:
   ```bash
   python pinch_to_servo_dual_udp_strict_sta.py --ip <ESP32_IP>
   ```
   Example:
   ```bash
   python pinch_to_servo_dual_udp_strict_sta.py --ip 192.168.0.57
   ```
3. You should see your camera feed and calibration bars.

---

### 3️⃣ Test Calibration
- Move fingers closer → servo goes toward **0°**
- Separate fingers → servo goes toward **180°**
- Use keys `Z/X` and `N/M` to fine-tune ranges.
- Press `Q` to exit.

---

## 🧰 File Overview
| File | Description |
|------|--------------|
| `pinch_to_servo_dual_udp_strict_sta.py` | Python controller (dual hand, UDP client) |
| `pinch_to_servo_dual_udp_strict_sta.ino` | ESP32 firmware for Wi-Fi UDP control |
| `README.txt` | Project overview |
| `demo_photo.png` | Optional visual demonstration |

---

## 📸 Demonstration
You can upload a short demo GIF or photo showing:
- OpenCV tracking window
- Two servos responding to finger distance in real-time

---

## 📜 License
MIT License © 2025 Mohammed Shehsin

---

## 👨‍💻 Author
**Mohammed Shehsin**  
Automation & Robotics Engineer  
GitHub: [github.com/Mohammed-Shehsin](https://github.com/Mohammed-Shehsin)  
LinkedIn: [linkedin.com/in/mohammed-shehsin](https://linkedin.com/in/mohammed-shehsin)

---

### Keywords
ESP32, Python, OpenCV, MediaPipe, Servo, Gesture Control, Robotics, IoT, UDP, Wi-Fi
