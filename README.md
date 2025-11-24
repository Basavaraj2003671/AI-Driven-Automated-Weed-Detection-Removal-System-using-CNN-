# AI-Driven-Automated-Weed-Detection-Removal-System-using-CNN-
This project presents an AI-powered embedded robotic system that automatically detects and removes weeds from agricultural fields using Convolutional Neural Networks (CNN), sensor fusion, and precision actuation. The system is designed to eliminate herbicide usage, protect crops, and automate weed management with high accuracy in real time.
The system integrates computer vision, LiDAR-based depth mapping, and a 2D linear actuator to physically remove weeds.
All processing happens on the edge, enabling real-time performance in outdoor farming conditions.

🔍 Key Features

Real-time weed vs crop classification using YOLOv11/YOLOv12 (CNN-based)

Camera + LiDAR fusion for accurate 3D localization

Precision mechanical weed removal using ESP32-controlled linear actuator

Fully embedded edge AI pipeline using NVIDIA Jetson Xavier NX

Herbicide-free automated weed management

Robust outdoor-ready electronics with custom power boards

🧠 System Workflow

1️⃣ Image Acquisition

A high-resolution camera captures live images of plants as the robot moves through crop rows.

2️⃣ Weed Detection using CNN (YOLOv11/YOLOv12)

Processed on NVIDIA Jetson Xavier NX

Custom-trained CNN model detects weed vs crop

Bounding box + pixel coordinates extracted in real time

3️⃣ Weed Localization (Camera + LiDAR Fusion)

RPLiDAR A2M12 collects distance data

Pixel coordinates + depth values are fused

Calculates real-world weed position (X, Y) relative to robot

4️⃣ Weed Removal Mechanism

Coordinates are sent to ESP32

A 2D linear actuator stage moves accurately to the weed position

A mechanical tool removes the weed without damaging nearby crops

5️⃣ Embedded Control System

Custom DC-DC converters & power management

Closed-loop alignment to maintain robot path

Reliable operation in harsh outdoor environments

🧩 High-Level System Architecture

Camera ──► Jetson NX ──► CNN (YOLOv11/12)
                     └─► Pixel Coordinates
LiDAR ────────────────► Distance Data
                     ▼
              Sensor Fusion
                     ▼
             Real-world Weed Location
                     ▼
                   ESP32
                     ▼
        2D Linear Actuator + End Effector
                     ▼
            Mechanical Weed Removal

🔧 Hardware Used

NVIDIA Jetson Xavier NX – AI inference

High-resolution camera – plant imaging

RPLiDAR A2M12 – distance mapping

ESP32 – actuator + low-level control

2D linear actuator system – physical weed removal

Custom power & DC-DC converter board

Outdoor-rated mechanical assembly

🧑‍💻 Software Stack
Component	Technology
CNN Model	YOLOv11 / YOLOv12
Jetson Code	Python, OpenCV, NumPy
Embedded Firmware	ESP32 (Arduino/FreeRTOS)
Fusion Algorithm	Camera–LiDAR calibration & mapping
Communication	UART / Serial
Visualization	Optional RViz / matplotlib
📊 Model Training (Short Summary)

Dataset collected from real crop fields

Annotated for crop and weed classes

Trained YOLOv11/YOLOv12 for fast CNN inference

Optimized using TensorRT for Jetson NX

Achieved strong outdoor detection accuracy


📌 Future Enhancements

Multi-row autonomous robot

Solar-powered version

Implement semantic segmentation (Mask-based weed removal)

IMU + RTK GPS–based navigation

Multi-tool end-effector for different weed types
