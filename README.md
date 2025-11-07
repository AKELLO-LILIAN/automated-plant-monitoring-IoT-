The proposed system automates monitoring and watering through integrated sensors and actuators. The soil moisture sensor continuously checks water levels, while the DHT11 tracks temperature and humidity. The Arduino analyses this data against predefined thresholds (e.g., soil moisture below 30% triggers watering). A water pump connected via a relay delivers precise amounts of water, creating a self-sustaining loop that maintains ideal conditions without human input. The LED is used an indicator for the pump during functionality.Functional Requirements
SENSOR MONITORING
• Soil Moisture Monitoring: The system shall read soil moisture levels every 10 minutes using the water level sensor.
• Environmental Monitoring: The system shall monitor temperature and humidity via the DHT11 sensor at the same interval.
IRRIGATION CONTROL
• Automatic Pump Activation: If soil moisture drops below a user-defined threshold (default: 40%), the system shall activate the water pump for a set duration (e.g., 10 seconds).
• Safe Relay Operation: The relay shall safely handle pump activation to prevent electrical issues or overload.
DATA MANAGEMENT
• Data Logging: The system shall log all sensor readings and actions to the serial output for debugging and potential future cloud integration.
POWER MANAGEMENT
• Energy Efficiency: The system shall operate on 5V USB or battery power and utilize sleep modes between readings to conserve energy.
STATUS INDICATION
• Status Indication: The system shall provide basic LED indicators to display operational status and alerts.
PROJECT DESIGN

Circuit Design:
DHT11 Sensor
• Signal (S): Connected to Digital Pin 2 of Arduino Uno
• VCC (+): Connected to 5V power supply
• GND (-): Connected to GND of Arduino Uno
Water Level Sensor
• Signal (S): Connected to Analog Pin A0 of Arduino Uno
• VCC (+): Connected to 5V power supply
• GND (-): Connected to GND of Arduino Uno
LED
• Anode: Connected to Digital Pin 13 via a 220Ω resistor
• Cathode: Connected to GND
Relay Module
• VCC: Connected to 5V power supply
• GND: Connected to GND of Arduino Uno
• IN1: Connected to Digital Pin 7 of Arduino Uno
How The System Works
The system operates in a continuous loop:
The Arduino Uno powers on and initializes all connected sensors, including the DHT11 and water level sensor. It begins monitoring environmental conditions and soil moisture automatically.
Every 10 minutes, the system reads the soil moisture level using the water level sensor, converting the analog signal to a percentage. Simultaneously, it acquires temperature and humidity data from the DHT11 sensor to assess environmental conditions.
If the soil moisture falls below a predefined threshold adjustable in the code and environmental factors such as temperature or humidity indicate increased water demand, the Arduino signals the relay module to activate the water pump for a fixed duration. The pump delivers water to the soil and then automatically shuts off. The LED turns on whenever the pump is on and off when the pump stops. It acts as an indicator for the pump operation.
Throughout operation, the system logs all readings and pump activity to the serial monitor for debugging and potential data analysis. The irrigation process is fully automated, requiring no human intervention after initial setup, while thresholds and timings can be modified directly in the firmware if needed.
