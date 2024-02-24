Here is a large assignment carried out in the Embedded System Design course with the topic of programming ESP32 on the ESP-IDF framework to operate a fire alarm, anti-theft, and automatic window opening system:

- The system uses ESP32 as the MCU, which is powered by 5V from the computer. Four sensors are employed: the flame infrared sensor KY-026, the water droplet sensor FC-37, the human infrared sensor PIR, and the DHT11 temperature/humidity sensor. ESP32 collects analog or digital signals from these sensors, makes control decisions for the alarm buzzer, SIM800L module, and sends 1/0 signals to L298N to control the electric cylinder motor. Our team uses L298N to control the electric cylinder motor for extension/retraction. After reaching the maximum travel distance, the electric cylinder motor stops automatically and consumes no additional power. The electric cylinder motor is a 12V type with a maximum power of 75W, powered by a 12V source from the bee's nest power supply.

- When both the FC-37 water droplet sensor signals water and the DHT11 temperature/humidity sensor signals high humidity to ESP32, ESP32 sends control signals to L298N to retract the window to prevent rain from entering the room.

- When both the KY-026 flame infrared sensor signals fire and the DHT11 temperature/humidity sensor signals high temperature to ESP32, ESP32 activates the alarm buzzer and controls the SIM module to send messages/calls to the homeowner/fire department.

- When the infrared sensor detects an intrusion, ESP32 activates the alarm buzzer and controls the SIM module to send messages/calls to the homeowner/police.
