# Budget-Bath-Project
This Arduino project automates bath filling by controlling hot and cold water solenoids. It ensures the desired bath temperature is achieved using flow sensors and user input via the Dabble app.

**Overview:**

**The system:** 
Accepts a target bath temperature from the user
Adjusts water flow using solenoid valves for hot and cold water
Uses flow sensors to measure water volume
Displays data on the Serial Monitor
Files:

**Final_Budget_Bath_Sketch.ino:**  
Main program to control the bath filling system.
Takes user input for temperature.
Manages solenoids and flow sensors to dispense the correct amount of hot and cold water.

**Pulse_Test_Calibration.ino:**  
Used to calibrate flow sensors, determining how many pulses equal 1 liter of water.

**How to Use:** 
Upload the Final_Budget_Bath_Sketch.ino to your Arduino.
Input your desired bath temperature using the Dabble app.
Monitor the system via the Serial Monitor to see water flow and solenoid status.

**Hardware Requirements:** 
Arduino, Hot and cold water solenoids, Water flow sensors, Dabble app for input
