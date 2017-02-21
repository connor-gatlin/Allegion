// https://www.arduino.cc/en/Tutorial/ReadWrite
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "ADXL345.h"


//******************************************************
// Global Variables
//******************************************************
// Sample rate of data collection in milliseconds
long sampleRate = 500;

// All pins connected for force readings
const int forcePin1 = A0;
const int forcePin2 = A1; 
const int forcePin3 = A2;
const int forcePin4 = A3;

// Accelerometer instance
ADXL345 adxl;

// Data File to be saved on SD card
File dataFile;

// Start time of data collection
long currTime = 0;

// Constant for input voltage from arduino (5V)
const float V_INPUT = 5.00;

// Constant for resistive divider (1K ohm)
const float RES_DIV = 100000;

// Approximate slope from calibration curve
const float CAL_SLOPE = 2155.5;
const float CAL_INTERCEPT = -0.5659;




//******************************************************
// setup() function called once.
// Used to intialize data file, SD card and Serial port.
//******************************************************
void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    // Wait for serial port to connect. Needed for native USB port only
  }

  // Initialize the pin modes for all force readings to INPUT
  pinMode(forcePin1, INPUT);
  pinMode(forcePin2, INPUT);
  pinMode(forcePin3, INPUT);
  pinMode(forcePin4, INPUT);

  // Initialize the accelerometer
  //initializeAccelerometer();

  if (sd)
  {
    // Initialize SD Card
    Serial.print("Initializing SD card...");
  
    if (!SD.begin(10)) {
      Serial.println("Initialization failed!");
      return;
    }
    Serial.println("Initialization done.");

    // Open data file for writing
    dataFile = SD.open("Data.txt", FILE_WRITE);
  
    // Ensure that the file opened
    if (dataFile) {
      Serial.print("Writing to Data.txt...");
      
      // Print header row
      dataFile.println("Time(ms),Force1(Lbs)");
      dataFile.close();
    } 
    else {
      Serial.println("ERROR opening test.txt");
    }
  }
}





//******************************************************
// loop() function called continuosly (based on sampleRate)
// Used to collect and perform initial analysis on force
// and acceleration readings. Prints to data.txt file
// on the SD card.
//******************************************************
void loop() 
{
  // Read in the raw voltage data
  int rawForce1 = analogRead(forcePin1);
  int rawForce2 = analogRead(forcePin2);
  int rawForce3 = analogRead(forcePin3);
  int rawForce4 = analogRead(forcePin4);

  // Use the raw analog force to calculate
  // voltage, resistance and force.
  float v1 = calcVoltage(rawForce1);
  float r1 = calcResistance(v1);
  float force1 = calcForce(r1);

  // Print out all information to Serial Monitor
  Serial.println(String(v1) + " V");
  Serial.println(String(r1) + " KOhms");
  Serial.println(String(force1) + " lbs");
  Serial.println();

  // Write to the SD card data file
  // Open the data log file and write to it
  dataFile = SD.open("Data.txt", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(String(currTime) + "," + String(force1));
    dataFile.close();
  }
  else
  {
    Serial.println("ERROR opening data file");
  }

  // Sample Rate
  Serial.println();
  delay(sampleRate);
  currTime += sampleRate;
}



float calcVoltage(float rawForce)
{
  return ((rawForce1 / 1023.0) * V_INPUT); // Returns voltage in volts
}



float calcResistance(float voltage)
{
  return (((V_INPUT / voltage) - 1) * RES_DIV) / 1000; // Returns resistance in kOhms
}



float calcForce(float resistance)
{
  float capacitance = 1 / resistance;
  return ((capacitance * CAL_SLOPE) + CAL_INTERCEPT);
}





//******************************************************
// initializeAccelerometer() function powers on and 
// initializes the accelerometer.
//******************************************************
void initializeAccelerometer()
{
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625μs per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
}
