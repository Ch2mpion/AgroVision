# AgroVision: Autonomous AI Plant Health Monitor 🌿🤖

**AgroVision** is an AI-IoT solution developed to transform greenhouse and agricultural monitoring. The system features an autonomous mobile platform (robotic vehicle) equipped with a camera that navigates through crop rows to perform real-time plant analysis and disease detection using a custom-trained **Convolutional Neural Network (CNN)**.

This project was proudly represented by the **Benha National University (BNU)** team during the **G-Force Round 2 Scientific Competition** at **Galala University**.

---

## ⚠️ Important Hardware Note (Lessons Learned)
During our extensive testing, we found that the **ESP32-CAM** sensor quality, high noise levels, and sensitivity to poor lighting conditions can significantly degrade the AI model's accuracy. 

**Recommendation:** For production-grade or highly reliable results, we strongly recommend using a **Raspberry Pi Camera** (e.g., Camera Module v2) paired with a Raspberry Pi board. The superior image clarity and sensor stability of the Raspberry Pi Cam provide a much more consistent input for the CNN model compared to the ESP32-CAM.

---

## 🚀 Project Overview
Manual crop inspection is often slow and inaccurate. **AgroVision** solves this by integrating:
1. **Autonomous Navigation:** A mobile chassis designed to traverse agricultural environments.
2. **AI Diagnostic Engine:** A **MobileNetV2-based CNN** model capable of identifying various plant diseases with high precision.
3. **Actionable Agriculture:** Instant identification of disease types to assist farmers in early intervention.

---

## 🛠️ Technical Stack
- **Artificial Intelligence:** TensorFlow, Keras, MobileNetV2 (Transfer Learning), Python.
- **Embedded Systems:** ESP32-CAM, Motor Drivers, Servo Motors.
- **Data Analysis:** Jupyter Notebooks (Google Colab).

---

## 📂 Repository Structure
* `arduino/`: Firmware for the ESP32-CAM and vehicle motor control logic.
* `notebooks/`: Includes `AgroVision.ipynb` containing the full training pipeline, data augmentation, and evaluation metrics.

---

## 🏆 Competition & Recognition
* **Event:** G-Force Round 2 - Scientific Competition.
* **Host:** Galala University.
* **Team Representation:** Benha National University (BNU).

### 👥 The BNU Team
* **Mahmoud Mohamed Mahmoud Abdeltawab**
* **Ahmed Mustafa Elbatanouny**
* **Ahmed Hassan Saleh**

**Supervised by:**
* **Dr. Mohamed Said**

---

## ❤️ Special Thanks
We would like to extend our deep gratitude to **Mohamed Badawy** for his invaluable technical guidance and support throughout the development of this project.
* **GitHub:** [M7MEDpro](https://github.com/M7MEDpro)

---

## ⚙️ Deployment & Usage (Optional)
This repository provides the core training pipeline to generate the `AgroVision.h5` model. Once you have generated the model file, deployment is flexible based on your needs:

1. **API Server Integration:** You can host the `.h5` model on a cloud server (e.g., using FastAPI and Ngrok) to perform remote inference from any device.
2. **Local Usage:** You can load the model directly into a local Python script or a custom GUI application for on-device processing.

*Note: This repository currently includes the training and hardware logic. Custom integration scripts or GUI wrappers are left to the user's preference and specific use case.*

---
*Developed with passion by the Engineering students of Benha National University.*
