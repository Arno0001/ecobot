\# EcoBot



\### Fire-Safe, Priority-Arbitrated Dual-ESP32 Robot for Garbage Detection and Obstacle Avoidance



\[!\[DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.23614673.svg)](https://doi.org/10.5281/zenodo.23614673)

\[!\[License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)



!\[EcoBot architecture](images/System\_archi.jpeg)



EcoBot is an autonomous ground robot built from two ESP32 boards that performs onboard garbage detection and sorting, fire alerting, and obstacle avoidance — all without any cloud or GPU. A priority arbiter (Fire > Garbage > Obstacle > Cruise) in the style of subsumption control keeps the robot safe while it works.



This repository contains the firmware source code and the full image dataset used to train the onboard classifier, released to accompany an IEEE conference paper submission.



\## Key Features



\- \*\*Dual-ESP32 architecture\*\* — one board (ESP32 DevKit) runs control, motor drive, and the safety-first priority arbiter; the other (ESP32-CAM) runs onboard vision.

\- \*\*TinyML garbage detection\*\* — a quantized (int8) Edge Impulse FOMO model classifies waste as biodegradable or non-biodegradable, entirely on-device.

\- \*\*Fire safety\*\* — an infrared flame sensor triggers an immediate motor lockout, overriding every other behavior.

\- \*\*Dual-sensor obstacle avoidance\*\* — ultrasonic (HC-SR04) and time-of-flight (VL53L0X) sensing, validated by an ablation study.

\- \*\*Mobile app control\*\* — a companion app connects over WebSocket for manual driving, live telemetry, and a fire-event log.



\## App



!\[Mobile app](images/App\_screenshot.jpeg)



\*Garbage Log and Fire Alert tabs.\*



\## Repository Structure



\- firmware/ecobot\_dev.ino — Main controller: arbiter, motors, sensors

\- firmware/ecobot\_cam.ino — ESP32-CAM: vision pipeline, FOMO inference

\- images/System\_archi.jpeg — Architecture diagram

\- images/App\_screenshot.jpeg — Mobile app screenshot

\- training/ — Training images (biodegradable / non-biodegradable / background)

\- testing/ — Held-out test images

\- info.labels — Bounding-box labels (Edge Impulse format)



\## Hardware



| Component | Role |

|---|---|

| ESP32 DevKit | Main controller, priority arbiter |

| ESP32-CAM (OV2640) | Onboard vision (FOMO inference) |

| HC-SR04 | Ultrasonic obstacle ranging |

| VL53L0X | Short-range ToF ranging |

| IR flame sensor | Fire detection |

| PCA9685 + 3× MG995 + SG90 | Claw, arm, base, scan servos |

| L298N + 4× DC gear motors | Locomotion |

| 3S + 2S 18650 packs, 2× LM2596 | Power |



\## Dataset



\- \*\*1,065 images\*\*, self-collected

\- Classes: `biodegradable`, `non-biodegradable`, `background`

\- Split: 852 training / 213 testing images

\- Format: bounding-box labels for object detection (Edge Impulse)



\## Results



| Metric | Result |

|---|---|

| Pick-and-sort accuracy | 34/40 correct (85.0%) |

| Fire-stop response (firmware) | 23.9 ms (≈325 ms incl. confirmation) |

| Obstacle stop (normal box) | 10/10, no contact |

| ToF ablation (low box, with ToF) | 14/14 self-stopped |

| ToF ablation (low box, without ToF) | 0/10 self-stopped |

| Flame detection range | \~10 cm |

| Battery life (AUTO mode) | 73 min |



\## Getting Started



1\. Flash firmware/ecobot\_dev.ino to the ESP32 DevKit and firmware/ecobot\_cam.ino to the ESP32-CAM using Arduino IDE.

2\. Set your Wi-Fi credentials in both sketches.

3\. Train your own FOMO model on the provided dataset via Edge Impulse, or use the included training/ and testing/ folders directly.



\## Citation



If you use this dataset or firmware, please cite:

