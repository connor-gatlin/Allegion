#include <SparkFun_ADXL345.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>


//******************************************************
// Global Variables
//******************************************************
// Sample rate of data collection in milliseconds
long sampleRate = 50;

// Boolean for sd card
bool sd = true;

// Accelerometer Calibration Values
#define offsetX   2       // OFFSET values
#define offsetY   9.5
#define offsetZ   -18.5

#define gainX     8.55        // GAIN factors
#define gainY     8.55
#define gainZ     7.84 

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

//Coefficient values for force sensor 1 calibration curve y = 73127x2 + 522.77x + 1.6106
const float CAL_S1_A2 = 73127;
const float CAL_S1_A1 = 522.77;
const float CAL_S1_A0 = 1.6106;

//Coefficient values for force sensor 2 calibration curve y = 44736x2 + 959.5x + 0.6082
const float CAL_S2_A2 = 44736;
const float CAL_S2_A1 = 959.5;
const float CAL_S2_A0 = 0.6082;

//Coefficient values for force sensor 3 calibration curve y = 44853x2 + 368.39x + 1.7637
const float CAL_S3_A2 = 44853;
const float CAL_S3_A1 = 368.39;
const float CAL_S3_A0 = 1.7637;

//Coefficient values for force sensor 4 calibration curve y = 28395x2 + 792.43x + 0.6379
const float CAL_S4_A2 = 28395;
const float CAL_S4_A1 = 792.43;
const float CAL_S4_A0 = 0.6379;


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
    SD.remove("Data.txt");
    dataFile = SD.open("Data.txt", FILE_WRITE);
  
    // Ensure that the file opened
    if (dataFile) {
      Serial.print("Writing to Data.txt...");
      
      // Print header row
      dataFile.println("Time(ms),Force1(Lbs),Force2(Lbs),Force3(Lbs),Force4(Lbs),X-Accel,Y-Accel,Z-Accel");
      dataFile.close();
    } 
    else {
      Serial.println("ERROR opening Data.txt");
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
  float v2 = calcVoltage(rawForce2);
  float v3 = calcVoltage(rawForce3);
  float v4 = calcVoltage(rawForce4);

  // Use the voltage to calculate resistance
  float r1 = calcResistance(v1);
  float r2 = calcResistance(v2);
  float r3 = calcResistance(v3);
  float r4 = calcResistance(v4);

  // Use the resistance to calculate the force in pounds
  float force1 = calcForce1(r1);
  float force2 = calcForce2(r2);
  float force3 = calcForce3(r3);
  float force4 = calcForce4(r4);

  // Print out all information to Serial Monitor
  Serial.println("Force 1 = " + printForce(force1));
  Serial.println("Force 2 = " + printForce(force2));
  Serial.println("Force 3 = " + printForce(force3));
  Serial.println("Force 4 = " + printForce(force4));

  // Accelerometer Readings
  int xAccel,yAccel,zAccel;
  // Read the accelerometer values and store them in variables declared above x,y,z
  adxl.readAccel(&xAccel, &yAccel, &zAccel);

  // Calculating new calibrated values for x,y,z accelerations
  int accX = (xAccel - offsetX)/gainX;
  int accY = (yAccel - offsetY)/gainY;
  int accZ = (zAccel - offsetZ)/gainZ;


  // Print Acceleration Values
  Serial.println(printAccel(accX, accY, accZ));
  

  // Write to the SD card data file
  // Open the data log file and write to it
  dataFile = SD.open("Data.txt", FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(String(currTime) + "," + String(force1) + "," + String(force2) + "," + String(force3) + "," + String(force4) + "," + printAccel(accX, accY, accZ));
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

float calcForce1(float resistance)
{
  float capacitance = 1 / resistance;
  //polynomial calibration curve y = 73127x2 + 522.77x + 1.6106
  return (CAL_S1_A2 * capacitance*capacitance) + (CAL_S1_A1 * capacitance) + CAL_S1_A0;
}

float calcForce2(float resistance)
{
  float capacitance = 1 / resistance;
  //polynomial calibration curve y = 57175x2 + 1081x + 0.1808
  return (CAL_S2_A2 * capacitance*capacitance) + (CAL_S2_A1 * capacitance) + CAL_S2_A0;
}

float calcForce3(float resistance)
{
  float capacitance = 1 / resistance;
  //polynomial calibration curve y = 44853x2 + 368.39x + 1.7637
  return (CAL_S3_A2 * capacitance*capacitance) + (CAL_S3_A1 * capacitance) + CAL_S3_A0;
}

float calcForce4(float resistance)
{
  float capacitance = 1 / resistance;
  //polynomial calibration curve y = 28395x2 + 792.43x + 0.6379
  return (CAL_S4_A2 * capacitance*capacitance) + (CAL_S4_A1 * capacitance) + CAL_S4_A0;
}



String printForce(float force)
{
   return String(force) + " lbs";
}



String printAccel(float x, float y, float z)
{
  return String(x) + "," + String(y) + "," + String(z);
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
  adxl.setRangeSetting(2);

  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
  // Default: Set to 1
  // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
  adxl.setSpiBit(0);
}
