## Balancia
# Overview
Balancia is a self-balancing robot project. This repository includes the implementation of fuzzy logic for balancing, the wiring diagram, flowchart, and analysis of the system using MATLAB.

# Features
1. Self-Balancing Robot powered by fuzzy logic.
2. MATLAB analysis for input, error, delta error, PWM, and results.
3. Includes wiring diagrams and system flowcharts.
4. Fully implemented on the Arduino platform using PlatformIO.

# Repository Contents
include/: Header files.
lib/: Libraries used for the robot.
src/: Main source code.
test/: Testing scripts.
platformio.ini: PlatformIO configuration.
README.md: Documentation.
Wiring Diagram: Circuit connections.
Flowchart System: Visual representation of the process.
MATLAB Analysis: Results and graphs for fuzzy logic inputs and output.


# Installation
# 1. Clone the repository
```
git clone https://github.com/ahmaadfaauzn2/Balancia.git
cd Balancia
```

# 2. Clone the repository
Set up PlatformIO
1. Install PlatformIO IDE on VS Code.
2. Open the project folder in VS Code.
3. Ensure platformio.ini is configured for your board (e.g., ESP32 or Arduino Uno).
 
# 3. Upload the code to the microcontroller
1. Connect your board via USB.
2. Run the following command in the terminal:
```
platformio run --target upload
```

# Key Code Snippets
1. Fuzzy Logic Implementation
```
float error = desired_angle - current_angle;
float delta_error = error - prev_error;

float pwm = calculateFuzzyPWM(error, delta_error);

motorControl(pwm);
prev_error = error;
```

2. Motor Control
```
void motorControl(float pwm) {
    if (pwm > 0) {
        analogWrite(motorA, pwm);
        analogWrite(motorB, 0);
    } else {
        analogWrite(motorA, 0);
        analogWrite(motorB, -pwm);
    }
}
```

3. Fuzzy Logic Rules
```
float calculateFuzzyPWM(float error, float deltaError) {
    // Define fuzzy rules and membership functions
    // Example:
    if (error > 0 && deltaError > 0) return HIGH_PWM;
    if (error < 0 && deltaError < 0) return LOW_PWM;
    // Add more rules as needed
    return 0;
}
```

# Results
# MATLAB analysis provides insights into:

1. Error and Delta Error
2. PWM adjustments
3. Fuzzy inference system results
4. View the MATLAB results in the MATLAB Input images in the repository.




# Wiring Diagram 
![wiring diagramm](https://github.com/user-attachments/assets/6243e3b0-f992-4148-a7ac-4121a37c9bbf)


# Flowchart System
![image](https://github.com/user-attachments/assets/ddea0280-1714-44f6-958f-20183f59582d)

# Matlab Input 1 (Error)
![image](https://github.com/user-attachments/assets/b2ac624c-ee5b-450a-a2cf-fed578d17712)

# Matlab Input 2 (Delta Error)
![image](https://github.com/user-attachments/assets/9b213322-ec25-48c7-988f-e72623df31ad)

# Matlab Input 3 (PWM)
![image](https://github.com/user-attachments/assets/8c23c2c7-a695-49c7-b63e-7bd18b5c57cf)

# Fuzzy Inference
![image](https://github.com/user-attachments/assets/8d9cb8df-35a0-44bb-80c4-d4b8c97d474b)

# Result Input 
![image](https://github.com/user-attachments/assets/9354dbbd-b693-46e8-9686-bd60561b2e64)


# Robot
![self robot](https://github.com/user-attachments/assets/90c9c5f1-b4d4-45f5-a430-4e9d36ce3070)

