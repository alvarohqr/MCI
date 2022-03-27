#include <MQ131.h>

void setup() {
  Serial.begin(115200);

  // Init the sensor
  // - Heater control on pin 6 ---> No connection
  // - Sensor analog read on pin P4_2 (2)
  // - Model LOW_CONCENTRATION
  // - Load resistance RL of 100kOhms (100000 Ohms)
  MQ131.begin(6,2, LOW_CONCENTRATION, 100000, (Stream *)&Serial);  

  Serial.println("Calibration in progress...");
  
  MQ131.calibrate();
  
  Serial.println("Calibration done!");
  Serial.print("R0 = ");
  Serial.print(MQ131.getR0());
  Serial.println(" Ohms");
  Serial.print("Time to heat = ");
  Serial.print(MQ131.getTimeToRead());
  Serial.println(" s");
}

void loop() {
  Serial.println("Sampling...");
  MQ131.sample();
  Serial.print("Concentration O3 : ");
  Serial.print(MQ131.getO3(PPM));
  Serial.println(" ppm");
  Serial.print("Concentration O3 : ");
  Serial.print(MQ131.getO3(PPB));
  Serial.println(" ppb");
  Serial.print("Concentration O3 : ");
  Serial.print(MQ131.getO3(MG_M3));
  Serial.println(" mg/m3");
  Serial.print("Concentration O3 : ");
  Serial.print(MQ131.getO3(UG_M3));
  Serial.println(" ug/m3");

  delay(60000);
}
