#include <SparkFun_ADXL345.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>


//******************************************************
// Global Variables
//******************************************************
// Sample rate of data collection in milliseconds
long sampleRate = 500;

// Boolean for sd card
bool sd = true;

// All pins connected for force//accel readings
const int forcePin1 = A0;
const int forcePin2 = A1; 
const int forcePin3 = A2;
const int forcePin4 = A3;
const int accelPin = A4;

// Accelerometer instance
ADXL345 adxl = ADXL345();

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
  initializeAccelerometer();

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
      dataFile.println("Time(ms),Force1(Lbs),X-Accel,Y-Accel,Z-Accel");
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

  // Use the raw analog force to calculate voltage
  float v1 = calcVoltage(rawForce1);
//  float v2 = calcVoltage(rawForce2);
//  float v3 = calcVoltage(rawForce3);
//  float v4 = calcVoltage(rawForce4);

  // Use the voltage to calculate resistance
  float r1 = calcResistance(v1);
//  float r2 = calcResistance(v2);
//  float r3 = calcResistance(v3);
//  float r4 = calcResistance(v4);

  // Use the resistance to calculate the force in pounds
  float force1 = calcForce(r1);
//  float force2 = calcForce(r2);
//  float force3 = calcForce(r3);
//  float force4 = calcForce(r4);

  // Print out all information to Serial Monitor
  Serial.println(printForce(force1));
//  Serial.println(printForce(force2));
//  Serial.println(printForce(force3));
//  Serial.println(printForce(force4));

  // Accelerometer Readings
  int xAccel,yAccel,zAccel;
  // Read the accelerometer values and store them in variables declared above x,y,z
  adxl.readAccel(&xAccel, &yAccel, &zAccel);

  // Print Acceleration Values
  Serial.println(printAccel(xAccel, yAccel, zAccel));
  

  // Write to the SD card data file
  // Open the data log file and write to it
  dataFile = SD.open("Data.txt", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(String(currTime) + "," + String(force1) + "," + printAccel(xAccel, yAccel, zAccel));
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
  return ((rawForce / 1023.0) * V_INPUT); // Returns voltage in volts
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



String printForce(float force)
{
   return String(force) + " lbs\n";
}



String printAccel(float x, float y, float z)
{
  return String(x) + "," + String(y) + "," + String(z) + "\n";
}





//******************************************************
// initializeAccelerometer() function powers on and 
// initializes the accelerometer.
//******************************************************
void initializeAccelerometer()
{
  // Power on the ADXL345
  adxl.powerOn();

  // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity
  adxl.setRangeSetting(8);

  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
  // Default: Set to 1
  // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
  adxl.setSpiBit(0);

  // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityXYZ(1, 0, 0);
  // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
  adxl.setActivityThreshold(75);

  // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityXYZ(1, 0, 0);
  // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setInactivityThreshold(75);
  // How many seconds of no activity is inactive?
  adxl.setTimeInactivity(10);

  // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setTapDetectionOnXYZ(0, 0, 1);
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
 
  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);" 
                                                        // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
                                                        // This library may have a problem using INT2 pin. Default to INT1 pin.
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(1);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(1);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);
}
