# Tennis Bot 🎾

## Overview
**Tennis Bot** is an ESP32-powered robot designed to automatically pick up tennis balls, eliminating the need to manually collect them during practice. It helps players save time and energy by autonomously or remotely moving toward tennis balls and storing them in its built-in compartment.

## Features
- **Bluetooth Control:** Operates via a Bluetooth-connected controller using the Bluepad32 library. No external batteries are needed for the controller connection.  
- **Motorized Drive System:** Dual-motor setup for precise movement — forward, reverse, and turning capabilities.  
- **Rolling Mechanism:** A roller system efficiently picks up tennis balls and stores them inside the bot.  
- **Efficient Power Use:** Motors are powered by a strong onboard battery for extended operation.  
- **All-Terrain Wheels:** Designed to move smoothly across various surfaces, making it suitable for both indoor and outdoor tennis courts.  

## How It Works
The ESP32 communicates with a Bluetooth controller that sends commands (e.g., move forward, reverse, turn, activate roller). Each button on the controller triggers specific movement or actions handled by the bot’s firmware.

## Technical Details
- **Microcontroller:** ESP32  
- **Library Used:** [Bluepad32](https://github.com/ricardoquesada/bluepad32) for Bluetooth gamepad support  
- **Motors:** 2 drive motors (left and right) + 1 roller mechanism  
- **Power:** Rechargeable battery for motors, USB or onboard power for ESP32  
- **Code File:** `tennisbot_firmware.iso`  
- **Language:** C++ (Arduino environment)

## Controls
| Action | Controller Button |
|--------|-------------------|
| Accelerate | X |
| Reverse / Brake | Square |
| Turn Left | L1 |
| Turn Right | R1 |
| Toggle Roller | Circle |

## Future Improvements
- Add object detection for autonomous ball retrieval  
- Integrate distance sensors for obstacle avoidance  
- Include battery level monitoring  

## Author
**Developed by:** Josue Castro  
**Project Collaboration:** Theta Tau – Xi Epsilon Chapter, Omicron Class  
**Institution:** California State University, Long Beach 

