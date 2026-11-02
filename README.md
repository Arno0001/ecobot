\# EcoBot — Fire-Safe, Priority-Arbitrated Dual-ESP32 Robot for Garbage Detection and Obstacle Avoidance



\[!\[DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23614673.svg)](https://doi.org/10.5281/zenodo.23614673)

\[!\[License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)



EcoBot is an autonomous ground robot built from two ESP32 boards that

performs onboard garbage detection and sorting, fire alerting, and

obstacle avoidance — all without any cloud or GPU. A priority arbiter

(Fire > Garbage > Obstacle > Cruise) in the style of subsumption

control keeps the robot safe while it works.



This repository contains the firmware source code and the full

image dataset used to train the onboard classifier, released to

accompany an IEEE conference paper submission.



!\[EcoBot architecture](images/System\_archi.jpeg)

\*System architecture: five functional layers from application to physical environment.\*



\## Key Features



\- \*\*Dual-ESP32 architecture\*\* — one board (ESP32 DevKit) runs control,

&#x20; motor drive, and the safety-first priority arbiter; the other

&#x20; (ESP32-CAM) runs onboard vision.

\- \*\*TinyML garbage detection\*\* — a quantized (int8) Edge Impulse FOMO

&#x20; model classifies waste as biodegradable or non-biodegradable,

&#x20; entirely on-device.

\- \*\*Fire safety\*\* — an infrared flame sensor triggers an immediate

&#x20; motor lockout, overriding every other behavior.

\- \*\*Dual-sensor obstacle avoidance\*\* — ultrasonic (HC-SR04) and

&#x20; time-of-flight (VL53L0X) sensing, validated by an ablation study.

\- \*\*Mobile app control\*\* — a companion app connects over WebSocket

&#x20; for manual driving, live telemetry, and a fire-event log.



\## Repository Structure

