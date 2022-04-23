#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <SHT1x.h>
#include "Adafruit_CCS811.h"
#include <Adafruit_BMP085.h> //se movieron archivos a libsx
#include <MQ131.h>
/*
 * SHT10  ----> T (°C) y HR (%)
 * CCS811 ----> CO2 y TVOC'S
 * BMP085 ----> Ti (°C) y P (hPa)
 * MQ-131 ----> Ozono (O3)
 */

#define MCI_MSP430FR5969LP 
//#define MCI_MSP430F5529 

#ifdef MCI_MSP430FR5969LP
  #define RFM95_RST   18  //P3.0
  #define RFM95_CS    11  //P1.3
  #define RFM95_DIO0  12  //P1.4
#elif  MCI_MSP430F5529
  #define RFM95_RST   28  //P7.0
  #define RFM95_CS    8   //P2.7
  #define RFM95_DIO0  5   //P1.6
#endif
// Specify data and clock connections and instantiate SHT1x object
#define dataPin  3  //P2.6
#define clockPin 19 //P1.2

//PIN MAP MSP430FR5969LP
/* RFM95W SPI           BMP085 y CCS811 I2C 
 * Vcc a 3.3V                 Vcc a 3.3V
   SCLK   P2.2    7           SCL   P3_5 
   MISO   P1.7    14          SDA   P3_6   
   MOSI   P1.6    15          
*/

//Estableciendo la comunicación con los sensores:
SHT1x sht1x(dataPin, clockPin);
Adafruit_CCS811 ccs; //I2C_ADDR ---> 0x5A
Adafruit_BMP085 bmp; //I2C_ADDR ---> 0x77

int counter = 0;
const long frequency = 915E6;  // LoRa Frequency
String msg = "";
char dev_id[12] = "Nodo3";

float temp, temp_i, hum, prsn, O3 = -1;
uint16_t eCO2, TVOC = -1;
long heatMQ131 = 0;
void setup() {
  Serial.begin(9600);

  init_ccs811();
  init_bmp085();
  init_mq131();
  init_lora();
  
}

void loop() {
  read_sht10();
  sleep(250);
  read_ccs811();
  sleep(250);
  read_bmp085();
  sleep(250);
  read_mq131();
  sender_lora();
}

void init_lora(){
  while (!Serial);

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_DIO0); //Selección de pines SPI y digital
  Serial.println("LoRa Sender");

  if (!LoRa.begin(frequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void sender_lora(){
  float lluvia = 3.2;
  float viento = 20;
  //El mensaje se manda como una cadena y se reconoce
  //cada elemento por su simbolo asociado
  msg =  String(dev_id) + "/" + String(temp) + "&" + String(hum)
                + "#" + String(prsn) + "@" + String(viento) + "$" + String(lluvia)
                + "^" + String(eCO2) + "!" + String(TVOC)+ "~" + String(O3);
  Serial.print("Enviando paquete #: ");
  Serial.println(counter);
  // Envío del paquete
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket(); 
  counter++;
  Serial.println(msg);
  sleep(5000);
}

void read_sht10() {
  // Lectura de los valores
  temp = sht1x.readTemperatureC();
  hum = sht1x.readHumidity();
}

void init_ccs811() {
  if (!ccs.begin()) {
    Serial.println("Failed to start sensor! Please check your wiring.");
    while (1);
  }
  //Calibracion en base al SHT10
  while (!ccs.available());
  ccs.setEnvironmentalData(hum, temp);
}

void read_ccs811() {
  if (ccs.available()) {
    if (!ccs.readData()) {
      eCO2 = ccs.geteCO2();
      TVOC = ccs.getTVOC();
    }
    else {
      Serial.println("ERROR!");
      while (1);
    }
  }
}

void init_bmp085(){
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
}
}

void read_bmp085(){ 
    temp_i = bmp.readTemperature(); 
    prsn = bmp.readPressure()/100; 
}

void init_mq131(){
  // Init the sensor
  // - Heater control on pin 2
  // - Sensor analog read on pin A0
  // - Model LOW_CONCENTRATION
  // - Load resistance RL of 1MOhms (1M Ohms)
  MQ131.begin(2,A0, LOW_CONCENTRATION, 1000000);  

  Serial.println("Calibration parameters");
  Serial.print("R0 = ");
  Serial.print(MQ131.getR0());
  Serial.println(" Ohms");
  Serial.print("Time to heat = ");
  Serial.print(MQ131.getTimeToRead());
  Serial.println(" s");
}

void read_mq131(){
  MQ131.sample();
  O3 = MQ131.getO3(PPM);
  heatMQ131 = MQ131.getTimeToRead();
  sleep(heatMQ131);
}
