#include <Wire.h>

int potPin = 15;  // ADC pin for the potentiometer on the ESP32

const int numReadings = 10; // Number of readings for averaging
int readings[numReadings];  // Array to store readings
int readIndex = 0;
int total = 0;
int average = 0;

void sendPotValue() {
    total -= readings[readIndex]; // Subtract the last reading
    readings[readIndex] = analogRead(potPin);  // Read the new value
    total += readings[readIndex]; // Add the new reading
    readIndex = (readIndex + 1) % numReadings; // Go to the next index

    average = total / numReadings;  // Calculate the average

    // Scale the potentiometer value to 0-180 degrees
    int scaledValue = (average * 180) / 4095; // Direct scaling
    Wire.write(scaledValue);
}

void setup() {
    for (int i = 0; i < numReadings; i++) readings[i] = 0; // Initialize readings
    Wire.begin(8);  // Address 8 with a 400 kHz I2C clock
    Wire.onRequest(sendPotValue);  // Register the request event handler

    pinMode(potPin, INPUT);  // Set potentiometer pin as input
}

void loop() {
  // Nothing to do in loop as we use onRequest to handle communication
}
