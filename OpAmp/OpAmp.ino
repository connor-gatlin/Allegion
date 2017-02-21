

const long calResistance = 8999, R2 = 100000;
const float calForceInLbs = 61.00;

const int fsrPin = A0;
const float kTimesTen = 9.1;
const float D = calForceInLbs * calResistance / R2 / (10 - kTimesTen) * 10;
float forceInLbs, forceInNewtons;

void setup() {
  pinMode(fsrPin, INPUT);
  Serial.begin(9600);
}

void loop() {

  //Serial.println(analogRead(fsrPin));
  float vOut = (analogRead(fsrPin) / 1023.0) * 5.0;

  float RFSR = (((5.0 / vOut) - 1) * 100000) / 1000;
  Serial.println(String(vOut) + " V");
  Serial.println(String(RFSR) + " KOhms");

  float calSlope = 2155.5;
  float calIntercept = -0.5659;

  float c = 1 / RFSR;

  float force = (c * calSlope) + calIntercept;
  Serial.println(String(force) + " lbs");
  Serial.println();

  /*
  forceInLbs = 10 *(D * (kTimesTen/10-(analogRead(fsrPin)/1023.0)));
  forceInNewtons = forceInLbs * 4.44822162;

  Serial.print("Force: \t");
  Serial.print(forceInLbs);
  Serial.print(" lbs. or ");
  Serial.print(forceInNewtons);
  Serial.print(" Newtons.");
  Serial.println();
  */
  delay(500);
}

/*

// Calibration Code
const int fsrPin = A0;
const float kTimesTen = 9.1;

long R2 = 100000, calResistance = 0;

void setup() {
  pinMode(fsrPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  calResistance = R2 * (10 - kTimesTen) / (kTimesTen - analogRead(fsrPin) * 10 / 1023) / 100;

  Serial.print("Use calResistance = ");
  Serial.print(calResistance);
  Serial.println();
  delay(500);
}
*/

