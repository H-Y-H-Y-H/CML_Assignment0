# Robot Circuit Design

The robot will be designed to use deep learning algorithms in real-time, and there are two approaches to implement this:
board computation and cloud computation. 
You have to choose one of these approaches and design a circuit accordingly. 


**Updates:** There is no limitation to the robot's functionality, so feel free to design the robot you are most familiar with. Also, I will be impressed if you list more than one robot with different circuits.


**Here are the details of the report you need to submit:**

Microcontroller: The microcontroller to be used in the circuit can be Arduino, Raspberry Pi, Jetson, Teensy, or any other 
microcontroller of your choice. 

Motors: The robot will have 12 motors. you will have to select advanced motors that you have used before and 
provide details such as brand, voltage, and other specifications. You should also be able to explain how 
to observe the state of the motors.

Communication Protocol: You will have to explain how they will communicate with the motors you select.
For example, you might use I2C, CAN BUS, or any other protocol of your choice.

Power Arrangement: You will have to design a PCB board for the power arrangement with a lipo-battery, 
taking into account the motors, sensors, and microcontrollers you will have to explain how you plan to provide power
to each component and ensure that the power supply is stable.

Sensors: In order to observe the position and orientation of the robot in the real world, you will have 
to select a sensor of your choice and provide details such as brand, voltage, and other specifications. 
Explain how you plan to integrate the sensor into the circuit and how they communicate with the microcontroller.

**Real-time Deep Reinforcement Learning**: To run real-time deep reinforcement
learning algorithms, you will have to design the circuit to speed up each step.
You can use techniques such as pipelining, parallelism, or other optimization techniques to ensure that the algorithm runs as fast as possible.


