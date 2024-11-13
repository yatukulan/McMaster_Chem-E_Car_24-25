# McMaster_Chem-E_Car_23-24
The codebase for the McMaster Chem-E Car Team from 2024-2025.

This project uses an Adafruit Feather RP2040 Adalogger microcontroller programmed in the Arduino programming language. The car runs off a chemical reaction and is stopped by a chemical reaction as well. However, the code and the microcontroller are used to start the stopping reaction and stir the mixture while monitoring it to identify when to stop based on a temperature threshold. It also uses an IMU to keep the car going straight, a servo for starting the stopping reaction, two motor drivers to run the drive motors, and a motor for stirring the stopping reaction. The car's main drive system will be powered from a custom-built Zn-Air battery while the remaining auxiliary systems are powered from a smaller Li-Po battery.

The major components used are as follows:<br />
2 - N20 100 RPM Motors (for drive system)<br />
1 - N20 1000 RPM Motor (for stirring mechanism)<br />
1 - SER0039 Servo Motor (for initially mixing reactants)<br />
2 - DRV8833 Motor Drivers<br />
1 - DS18B20 Temperature Sensor (for  temperature compensation in conductivity readings)<br />
1 - DFR0300-H Conductivity Probe (for monitoring stopping reaction progress)<br />
1 - Adafruit Feather RP2040 Adalogger Microcontroller<br />
1 - 1200 mAh Li-Po battery<br />

This repository also contains the custom-made PCB design for the component wiring we are using this year. The design was made using Altium, source files & gerber files are present in this repository.

The main objective of the competition is to design a small car to carry a certain amount of weight a set distance while powering and stopping the vehicle with a chemical reaction. The distance and weight vary each year. The compectition is hosted by the American Institute of Chemical Engineers (AIChE).
