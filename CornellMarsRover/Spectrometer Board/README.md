# Spectrometer Board

## Overview
The Spectrometer Board is a custom-made circuit designed for the Cornell Mars Rover Project to help analyze soil and rock samples. It works by shining light on the samples and then measuring the light that passes through using a photodiode.

The board has LEDs to light up the samples and a photodiode to detect how much light is reflected or transmitted. An op-amp on the board boosts the current from the photodiode and converts it into a measurable voltage. This voltage gives the science team useful info about what the samples are made of and their optical properties.

---

## Key Features
- Photodiode sensor for precise analog light intensity measurement  
- Op-amp signal conditioning stage for current-to-voltage conversion and amplification  
- LED driver circuit for controlled and consistent sample illumination  
- Amplified voltage output proportional to detected light intensity  
- Uses a STM32G431CBT6 microcontroller  
- CAN FD communication interface for integration with the rover’s main control network  
- 28 V power input with onboard regulation for 5 V and 3.3 V rails  

---

## Function
1. Light Emission:
   The onboard LEDs emit light through a soil or rock sample that's in its path.

2. Detection:
   The photodiode reads the intensity of the light, inducing a current that matches how much light there is.

3. Filtering and Amplification:
   The onboard op-amp converts and amplifies the photodiode’s current into a stable voltage output, filtered for noise reduction and precision.

4. Data Transmission:
   The STM32G431CBT6 microcontroller takes in the conditioned voltage signal and sends the measurement data to the rover’s NVIDIA Jetson Orin Nano using CAN FD. The science team then uses this info for real-time material analysis.
---
