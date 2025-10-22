# 🛏️ Smart Bed Control System  

### Intelligent Motorized Bed Adjustment using Eye-Blink Detection and GUI Control  

---

## 📁 Project Overview

This project implements **two control mechanisms** for a motorized smart bed using an **ESP32** as the controller.  
Both methods allow the user to adjust parts of the bed using different interaction modes — either through **eye blink detection** or a **graphical user interface (GUI)**.  

The serial output from the ESP32 provides live status updates of the bed’s current state.  
> ⚙️ *Note:* The serial feedback is currently used **only for debugging** and will later be replaced by a more refined feedback method.

---

## 🧠 Control Methods

### **A. Eye Blink Detection Control**
- **Folder:** `src/bed/`
- **Python Script:** `blink2.py`
- Uses computer vision to detect eye blinks and translate them into control commands for bed adjustment.
- Sends corresponding commands to the ESP32 via serial communication.

---

### **B. GUI-Based Control**
- **Folder:** `src/bedgui/`
- **Python Script:** `blink2_gui.py`
- Built using **PyQt** for an interactive and user-friendly interface.
- Allows manual control of bed position through buttons and visual indicators.

---

## 🖥️ System Behavior

- The ESP32 continuously sends **serial output** indicating:
  - Current motor state  
  - Bed position status  
  - Control inputs received (from blink detection or GUI)

> Example:  
> ```
> [STATUS] Bed Head Raised  
> [INPUT] Blink Detected → Lower Leg Section  
> ```

---

## 🧩 Folder Structure

```
SmartBed/
│
├── src/
│   ├── bed/               # Code for eye blink detection control
│   │   └── main.cpp       # ESP32 firmware for blink control
│   │
│   ├── bedgui/            # Code for GUI-based control
│   │   └── main.cpp       # ESP32 firmware for GUI control
│
├── blink2.py              # Python script for blink detection
├── blink2_gui.py          # Python script for GUI interface
└── README.md
```

---

## ⚙️ How to Run

### **1. Blink Detection Mode**
1. Navigate to the `bed` folder.  
   ```bash
   cd src/bed
   ```
2. Upload the firmware to your ESP32 using PlatformIO.  
3. Run the blink detection script:  
   ```bash
   python blink2.py
   ```

---

### **2. GUI Control Mode**
1. Navigate to the `bedgui` folder.  
   ```bash
   cd src/bedgui
   ```
2. Upload the firmware to your ESP32 using PlatformIO.  
3. Run the GUI script:  
   ```bash
   python blink2_gui.py
   ```

---

## 🧪 Debugging

- Open the **Serial Monitor** in PlatformIO or any terminal (baud rate: *115200*).  
- Observe real-time logs and status updates for:
  - Motor position
  - Blink detection events
  - GUI command feedback

---

## 🚧 Current Status

- [x] Blink detection functional  
- [x] GUI-based control working  
- [ ] Wireless communication feedback  
- [ ] Sensor-based automatic positioning  

---

## 🌟 Future Enhancements

- Integrate a dedicated display for real-time status (e.g., Nextion or OLED)
- Add wireless/mobile control via Wi-Fi or Bluetooth  
- Implement feedback sensors for precise angle monitoring  
- Introduce safety features and user calibration options  

---

## 🧰 Tech Stack

| Component | Description |
|------------|-------------|
| **ESP32** | Motor control and serial communication |
| **Python (OpenCV, PyQt)** | Blink detection and GUI interface |
| **PlatformIO** | Firmware development and deployment |
| **Serial Monitor** | Temporary debugging feedback channel |

---

## ✨ Acknowledgements

Developed as part of an assistive automation project focused on **accessible control systems** and **smart rehabilitation technology**.  
