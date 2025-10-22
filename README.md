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

