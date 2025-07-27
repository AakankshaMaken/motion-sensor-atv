# 🛵 Gesture-Controlled ATV using Arduino & nRF24L01

Control an All-Terrain Vehicle (ATV) wirelessly using **hand gestures **, built with Arduino and the **nRF24L01 RF module**. This project reads motion/joystick input on a glove (transmitter) and sends commands to the ATV (receiver), controlling its movement.

---

## 📸 Demo

> *(Insert a video or GIF showing the ATV responding to hand gestures)*  
> Example:  
> ![Demo](images/demo.gif)

---

## 🧠 Project Overview

| Component      | Description                                    |
|----------------|------------------------------------------------|
| Transmitter    | Worn on the hand. Captures X/Y analog values and transmits via nRF24L01 |
| Receiver       | Mounted on ATV. Receives X/Y data and drives motors |
| Communication  | nRF24L01 (2.4GHz wireless) |
| Motor Driver   | L298N |
| Range          | 20–50 meters typical |
| Power          | 9V battery or power bank |

---

## ⚙️ Hardware Used

- Arduino Uno / Nano (×2)
- nRF24L01 modules (×2)
- Joystick / Accelerometer (MPU6050)
- L298N Motor Driver
- 2 DC motors with ATV chassis
- Breadboard, jumper wires
- Capacitor (10µF–100µF for nRF stability)
- Power supply

---

## 📁 Project Structure

gesture-atv/
├── transmitter/
│ └── transmitter.ino # Sends analog data wirelessly
├── receiver/
│ └── receiver.ino # Receives and processes motor control
├── images/
│ └── circuit-diagram.png # Optional circuit diagram
├── README.md


---

## 📟 Transmitter Code

```cpp
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const int x_out = A0;
const int y_out = A1;
RF24 radio(8, 10);
const byte address[6] = "00001";

struct data {
  int xAxis;
  int yAxis;
};
data send_data;

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
}

void loop() {
  send_data.xAxis = analogRead(x_out);
  send_data.yAxis = analogRead(y_out);
  radio.write(&send_data, sizeof(data));
}

// Arduino Gesture Control Robot
// Receiver Circuit
// Created by DIY Builder

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Create RF24 radio object with CE = pin 8, CSN = pin 10
RF24 radio(8, 10);
const byte address[6] = "00001"; // Pipe address for communication

// Structure of incoming data from transmitter
struct data {
  int xAxis;
  int yAxis;
};
data receive_data;

// Motor pins (connected to L298N motor driver)
const int in1 = 5;  // Left motor forward
const int in2 = 6;  // Left motor backward
const int in3 = 9;  // Right motor forward
const int in4 = 10; // Right motor backward

void setup() {
  // Start serial monitor (optional for debugging)
  Serial.begin(9600);

  // Set motor control pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize RF24 communication
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);       // Low power to avoid interference
  radio.setDataRate(RF24_250KBPS);     // Lower speed = more stable range
  radio.startListening();              // Set to receive mode
}

void loop() {
  // If data is available from the transmitter
  if (radio.available()) {
    // Read incoming data into receive_data struct
    radio.read(&receive_data, sizeof(receive_data));

    int xAxis = receive_data.xAxis;
    int yAxis = receive_data.yAxis;

    // Debugging output (can be removed later)
    Serial.print("X: "); Serial.print(xAxis);
    Serial.print(" | Y: "); Serial.println(yAxis);

    // Movement logic based on X and Y axis values
    if (xAxis < 300) {
      // LEFT turn
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    } else if (xAxis > 700) {
      // RIGHT turn
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    } else if (yAxis > 700) {
      // FORWARD
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    } else if (yAxis < 300) {
      // BACKWARD
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    } else {
      // STOP
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }
  }
}



