#include <Arduino.h>
#include <Wire.h>  // Wire library - used for I2C communication
#include <SoftwareSerial.h>
#include "TinyGPS++.h"
#include <AltSoftSerial.h>

// Declare a TinyGPSPlus object
TinyGPSPlus gps;

AltSoftSerial SIM;

//altSerial SIM(11, 10); // SIM800L Tx & Rx is connected to Arduino #11 & #10
SoftwareSerial Serial1(4, 3); // Air530 GPS module Tx & Rx is connected to Arduino #4 & #3

int ADXL345 = 0x53; // The ADXL345 sensor I2C address
float X_out, Y_out, Z_out, Magnitude, X_off = 0, Y_off = 0, Z_off = 0;  // Outputs
float Threshold = 0.05; // Minimum acceleration required to start counting distance (adjust as needed)
float distance = 0.0;

unsigned long previousTime = 0;
const unsigned long MIN_UPDATE_INTERVAL = 6 * 60 * 60 * 1000; // Minimum interval between updates (6 hours)

void updateSerial()
{
  delay(50);
  while (Serial.available())
  {
    SIM.write(Serial.read()); // Forward what Serial received to Software Serial Port
  }
  while (SIM.available())
  {
    Serial.write(SIM.read()); // Forward what Software Serial received to Serial Port
  }
}

void setup()
{
  Serial.begin(9600); // Initiate serial communication for printing the results on the Serial monitor
  Serial1.begin(9600);
  SIM.begin(9600);

  Serial.println(F("Arduino - Tracking device"));

  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable
  Wire.endTransmission();

  Serial.println("Initializing...");
  delay(1000);

  SIM.println("AT"); // Once the handshake test is successful, it will back to OK
  updateSerial();
  SIM.println("AT+CSQ"); // Signal quality test, value range is 0-31, 31 is the best
  updateSerial();
  SIM.println("AT+CCID"); // Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  SIM.println("AT+CREG?"); // Check whether it has registered in the network
  updateSerial();

  // SIM.println("AT+CSCLK=2"); // Sleep mode 2
  // updateSerial();

  for (int i = 0; i < 10; i++) // To find out the zero-g offset values for each axis based on 10 measurements
  {
    Wire.beginTransmission(ADXL345);
    Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    X_off += (Wire.read() | Wire.read() << 8); // Sum of X-axis values of acceleration
    Y_off += (Wire.read() | Wire.read() << 8); // Sum of Y-axis values of acceleration
    Z_off += (Wire.read() | Wire.read() << 8); // Sum of Z-axis values of acceleration
    delay(200);
  }

  Wire.endTransmission();

  X_off /= 256 * 10;
  Y_off /= 256 * 10;
  Z_off /= 256 * 10;
}

void loop()
{
  // === Read accelerometer data === //
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = (Wire.read() | Wire.read() << 8); // X-axis value
  X_out = X_out / 256 - X_off; // For a range of +-2g, divide the raw values by 256 and account for non-zero g offset
  Y_out = (Wire.read() | Wire.read() << 8); // Y-axis value
  Y_out = Y_out / 256 - Y_off;
  Z_out = (Wire.read() | Wire.read() << 8); // Z-axis value
  Z_out = Z_out / 256 - Z_off;

  Magnitude = sqrt(pow(X_out, 2) + pow(Y_out, 2) + pow(Z_out, 2));

  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - previousTime; // Calculate time difference

  //Calculate the change in velocity
  float AccDiff = Magnitude - Threshold;
  float velocity = 0.0;
  if (AccDiff > 0)
  {
    velocity = AccDiff * (timeDiff / 1000.0); // Convert timeDiff to seconds
  }

  //Calculate the distance covered
  distance += velocity * (timeDiff / 1000.0); // Convert timeDiff to seconds

  //Update previous time
  previousTime = currentTime;

  Serial.print("Magnitude = ");
  Serial.println(Magnitude);
  Serial.print("Distance = ");
  Serial.println(distance);

  while(distance >= 4 || timeDiff >= MIN_UPDATE_INTERVAL)
  {
    // Wake up GSM module from sleep
    // SIM.println("AT");
    // SIM.println("AT+CSCLK=0");
    // updateSerial();

    // Get GPS data
    while(Serial1.available())
    {
      if (gps.encode(Serial1.read()))
      {
        Serial.print("Yeee\n");
        String msg = Serial1.readStringUntil('\r');
        Serial.println(msg);
        Serial.print("Current Location:\n");
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
        Serial.write(26); // End of message character

        if(gps.location.isValid())
        {
          Serial.print("DAMN!");
          // Send SMS with current location
          SIM.println("AT+CMGF=1"); // Configuring TEXT mode
          updateSerial();
          SIM.println("AT+CMGS=\"+ZZxxxxxxxxx\""); // Change ZZ with country code   and xxxxxxxxxxx with phone number to send SMS
          updateSerial();
          String msg = Serial1.readStringUntil('\r');
          SIM.println(msg);
          SIM.print("Current Location:\n");
          SIM.print("Latitude: ");
          SIM.println(gps.location.lat(), 6);
          SIM.print("Longitude: ");
          SIM.println(gps.location.lng(), 6);
          SIM.write(26); // End of message character
          updateSerial();
          Serial.println("SMS sent with current location.");

          // Put GSM module back to sleep
          // SIM.println("AT+CSCLK=2"); // Sleep mode 2
          // updateSerial();

          distance = 0.00;
          timeDiff = 0.00;
        }
      }
    }
  }
 updateSerial();
 delay(100);
}
