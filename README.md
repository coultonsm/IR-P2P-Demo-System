# Infrared Point to Point Demo System
This project was created for my final in Internet of Things at Clark University.

It implements a simple protocol seen below to exchange sensor information between two different Arduino platforms.

## The Protocol
<img width="900" height="516" alt="protocol_graph" src="https://github.com/user-attachments/assets/9edfe927-19ad-4b15-861e-946035665f3c" />

Each command is a two-byte value that is sent using the NEC protocol built-in to the Arduino-IRremote library.

This protocol allows for each device to have its own address, creating easy separation of messages for the receivers. 
The Arduino UNO in the demo has address 0x01 and a light sensor, and the MEGA board has address 0x02 and a DHT11 sensor.

The NEC protocol also has simple built in error detection using a logical NOT, sending normal bits first then inverted bits.

## Implementation of the Protocol
### Arduino UNO (Head node)
Below is a diagram of the implementation of the protocol for the UNO board / head node. It reports its own sensor information and gets DHT11 sensor readings from the MEGA. It has extra functionality required for the middlebox, allowing for a pub-sub notification model.
<img width="897" height="1055" alt="uno_fsm" src="https://github.com/user-attachments/assets/de43d215-cde1-48d4-8888-092d10cb9bff" />

### Arduino MEGA (Sensor-only child node)
Below is a diagram of the implementation of the protocol for the MEGA board / child node. It reports its own sensor information and gets light sensor readings from the UNO.
<img width="428" height="694" alt="mega_fsm" src="https://github.com/user-attachments/assets/04e0ff90-60a5-414b-a4b3-a7f729ea0f0c" />

## Presentation & Demo Video
https://www.youtube.com/watch?v=3OmpZed7EbA
