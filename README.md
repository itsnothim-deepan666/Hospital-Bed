# Autonomous Hospital Bed

## Technical Documentation and Developer Guide

---

# 1. Introduction

The Autonomous Hospital Bed is an embedded robotic assistive system designed to automate patient repositioning, reduce caregiver workload, and serve as a foundation for intelligent healthcare automation.

The system currently focuses on reliable multi-axis bed articulation using electrically actuated mechanisms, closed-loop angular feedback, and a touchscreen interface for testing and calibration.

The project is currently in:

**TRL-4 — Laboratory Prototype Validation**

This documentation serves as:

* Internal technical documentation
* Developer onboarding guide
* Firmware/software reference
* Future development roadmap

---

# 2. Problem Statement

Traditional hospital beds require manual repositioning of patients. This creates several challenges:

* High physical dependency on caregivers
* Delayed posture correction
* Increased risk of pressure ulcers
* Limited adaptability for patient conditions
* No integration with intelligent monitoring systems

The objective of this project is to develop a robotic bed system capable of controlled repositioning and future autonomous decision-making.

---

# 3. Objectives

## Current Objectives

* Automate backrest movement
* Automate legrest movement
* Automate side tilting
* Implement reliable angle-based positioning
* Build stable actuator stopping logic
* Establish feedback-based control

---

## Future Objectives

* Posture correction automation
* Pressure ulcer prevention
* Vital-triggered repositioning
* Fall detection
* Nurse alerting
* Vision-assisted monitoring
* Autonomous patient interaction

---

# 4. System Overview

The bed currently uses three actuators for mechanical articulation.

## Implemented Motions

| Motion            | Status          |
| ----------------- | --------------- |
| Backrest Up/Down  | Implemented     |
| Legrest Up/Down   | Implemented     |
| Left Tilt         | Implemented     |
| Right Tilt        | Implemented     |
| Height Adjustment | Not Implemented |

Current system capabilities:

* Manual directional control
* Preset angle control
* WiFi trigger reception
* Closed-loop motion stopping
* Experimental encoder integration

---

# 5. Mechanical Architecture

## Actuator Layout

The system uses **3 actuators**.

---

## Actuator 1 — Backrest

Mechanism:

Lead-screw

Purpose:

Controls upper-body elevation.

Movement:

* Up
* Down

---

## Actuator 2 — Legrest

Mechanism:

Lead-screw

Purpose:

Controls lower-body elevation.

Movement:

* Up
* Down

---

## Actuator 3 — Side Tilt

Mechanism:

Half/full gear chain mechanism

Purpose:

Controls left-right lateral bed tilt.

Movement:

* Left tilt
* Right tilt

---

## Current Limitations

Height adjustment is not yet implemented.

Mechanical specifications such as:

* Stroke length
* Speed
* Load capacity

are still under finalization.

---

# 6. Electronics Architecture

## Main Controller

### ESP32

ESP32 is the primary microcontroller.

Selected because of:

* Multiple UART channels
* High GPIO availability
* Built-in WiFi
* Fast interrupt handling
* Better scalability

UART availability is important for:

* Nextion display communication
* Serial debugging
* Future Raspberry Pi communication

---

## Motor Drivers

### Cytron Motor Drivers

Used for driving all actuator motors.

Functions:

* Bidirectional motion
* PWM speed control
* Load-capable switching

---

## Power Architecture

Current prototyping path:

DC Source → Voltage Regulator → Cytron Driver → DC Motor

This power architecture is still experimental.

---

# 7. Feedback System

The project currently uses a hybrid feedback architecture during transition.

---

## Phase 1 — IMU-Based Feedback

Initial implementation used:

MPU6050 × 3

Purpose:

Angular measurement of moving bed sections.

---

### IMU Placement

#### IMU 1

Location:

Backrest moving section

Purpose:

Backrest angle feedback

---

#### IMU 2

Location:

Legrest moving section

Purpose:

Legrest angle feedback

---

#### IMU 3

Location:

Side tilt half-gear mechanism

Purpose:

Side tilt debugging

Note:

This placement is experimental and not final.

---

## I2C Multiplexer

### TCA9548A

Used to handle multiple MPU6050 sensors with identical addresses.

Functions:

* Channel isolation
* Address conflict resolution
* Controlled sensor switching

---

## IMU Limitations

Observed issues:

* Communication issues through long wires due to I2C based communication
* Angle noise

This caused migration toward encoders.

---

# 8. Encoder Integration

The system is transitioning to encoder-based position feedback.

---

## Current Encoder

Type:

Magnetic incremental encoder

Current state:

Testing phase

Purpose:

Angular position tracking

Used for:

* Active closed-loop feedback
* Position-based stopping

---

## Current Limitations

Current encoder is temporary.

Problems:

* Mechanical placement complexity
* Resolution uncertainty
* Long-term wear concerns

---

## Future Recommendation

Planned upgrade:

* Absolute rotary encoder
  or
* Linear feedback actuator

Reason:

Improves:

* Startup consistency
* Position memory
* Safety

---

# 9. Control System

The bed currently uses threshold-based closed-loop control.

## Basic Logic

1. Receive command
2. Assign target
3. Read feedback
4. Compute error
5. Drive actuator
6. Stop at threshold

---

## Control Type

Current:

Threshold-based stopping

Future:

PID-based position control

---

# 10. User Interface

## Display

Nextion Display:

NX8048T050_011

Used for:

* Motion testing
* Preset validation
* Calibration
* Debugging

---

## Page 0 — Manual Directional Control

Buttons:

* TOP
* BOTTOM
* LEFT
* RIGHT

Purpose:

Direct motion control.

Used during:

* Mechanical testing
* Direction validation
* Wiring verification

Flow:

Button → ESP32 → Motor Driver → Actuator Motion

---

## Page 1 — Preset Angle Control

Buttons:

* 0°
* 30°
* 45°
* 60°
* MENU

Purpose:

Move bed sections to predefined angles.

Flow:

Preset → Target Assignment → Feedback Loop → Stop

This is the base of future autonomous posture control.

---

## Preset State Mapping

| State          | Angle |
| -------------- | ----- |
| Neutral        | 0°    |
| Low Incline    | 30°   |
| Medium Incline | 45°   |
| High Incline   | 60°   |

These states will later be called internally by autonomous modules.

---

# 11. Firmware and Software Environment

## Embedded Firmware

The firmware is developed using:

### PlatformIO

PlatformIO is used for:

* Building firmware
* Library management
* Flashing ESP32
* Serial monitoring

All embedded logic must be written inside:

```cpp id="4h9gtv"
src/main.cpp
```

This file currently contains:

* Motion logic
* Sensor logic
* UI handling
* Communication handling
* Stopping logic

Future modularization may separate this.

---

## Required Libraries

Current libraries:

* Adafruit_BusIO
* Adafruit_MPU6050
* Adafruit_Unified_Sensor
* I2Cdev
* MPU6050
* Nextion

---

# 12. Blink Detection Subsystem

The system includes a vision-based blink detection layer.

This acts as an external trigger interface.

---

## blink.py

Purpose:

Detect blinks and send UDP triggers.

Functions:

* Camera capture
* Blink detection
* UDP transmission

Flow:

Camera → Detection → UDP → ESP32

Used for lightweight testing.

---

## blink_gui.py

Purpose:

Blink detection with real-time PyQt GUI visualization.

Functions:

* Detection
* Live GUI display
* UDP command transmission

Flow:

Camera → Detection → GUI + UDP

Used for debugging and live monitoring.

---

## Communication Protocol

Current:

UDP over WiFi

Used for:

* Trigger communication
* Blink event transmission

Future:

Raspberry Pi local communication with ESP32

This removes WiFi dependency.

---

# 13. Communication Architecture

Current communication:

WiFi trigger reception

Used for:

* Remote debugging
* Blink event triggering

Planned:

Camera → Raspberry Pi → Local Processing → ESP32

This forms the future autonomy pipeline.

---

# 14. Future Autonomous Features

Planned:

---

## Posture Correction

Automatically detect and correct posture.

---

## Pressure Ulcer Prevention

Timed repositioning.

Example:

30° tilt after fixed intervals.

---

## Vital-triggered Repositioning

Planned sensors:

* MAX30102
* MLX90614
* AD8232

Used for adaptive posture decisions.

---

## Fall Detection

Detect unsafe patient movement.

---

## Nurse Alerting

Emergency notifications.

---

# 15. Safety Requirements

Current prototype lacks complete safety architecture.

Must add:

* Hardware emergency stop
* Software timeout
* Angle limits
* Sensor failure fallback
* Stall detection
* Current overload detection

This system is human-facing.

Safety is mandatory.

---

# 16. Current Limitations

* Encoder integration incomplete
* Height adjustment absent
* IMU system still partially active
* Side encoder architecture not finalized
* Safety layer incomplete
* No patient load testing
* No full autonomy layer

---

# 17. Development Roadmap

## Phase 1

Stabilize encoder system

---

## Phase 2

Fully replace IMU feedback

---

## Phase 3

Integrate Raspberry Pi vision system

---

## Phase 4

Add biomedical sensors

---

## Phase 5

Build autonomous posture engine

---

# 18. Developer Notes

Before modifying the system:

1. Use PlatformIO only.
2. Keep firmware logic inside `src/main.cpp`.
3. Verify motor polarity before testing.
4. Validate sensor readings before actuation.
5. Test encoder calibration.
6. Test without patient load.
7. Never deploy without emergency stop.

This is a safety-critical robotic system.

Every firmware change directly affects physical motion and must be validated before deployment.
