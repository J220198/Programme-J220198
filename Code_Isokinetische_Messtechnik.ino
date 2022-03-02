//DATA
String Data;
int XXX = 0;

//CO2 Sensor
#include "MHZ19.h"
#include <SoftwareSerial.h>

#define RX_PIN 8
#define TX_PIN 9
#define BAUDRATE 9600

MHZ19 myMHZ19;
SoftwareSerial mySerial (RX_PIN, TX_PIN);

unsigned long timeElapse = 0;

void verifyRange(int range);

//CO Sensor
int Messpin = A0;
int CO;

//NO2 Sensor
#include <SPI.h>
unsigned int result = 0;
byte byteOne = 0;
byte byteTwo = 0;
float percent = 0;


void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 
 //NO2
 SPI.beginTransaction(SPISettings(1500000, MSBFIRST, SPI_MODE0));
 SPI.begin();
 
 //CO2 Sensor
 mySerial.begin(BAUDRATE);
 myMHZ19.begin(mySerial);

 myMHZ19.autoCalibration(false);
 Serial.print("ABC Status: ");  myMHZ19.getABC() ? Serial.println("ON") : Serial.println("OFF");
 timeElapse = 400000;
 Serial.print("Wait ");
 Serial.print(timeElapse/60000);
 Serial.print(" Minutes to stabalise and Warm up");
 
 //while(millis() < timeElapse) {};

 Serial.println("Calibrating CO2 Sensor");
 myMHZ19.calibrate();

}

void loop() {
  // put your main code here, to run repeatedly:
  
  //CO2
  int CO2;
  CO2 = myMHZ19.getCO2();
  timeElapse = millis();

  //CO

  CO = analogRead(Messpin);

  //NO2

  unsigned int reading = 0;
  digitalWrite(SS, LOW);
  reading = SPI.transfer16(0x0000);
  digitalWrite(SS, HIGH);

  reading = reading << 3;
  reading = reading >> 4;

  float NO2 = reading * (5.000 / 4095);
  
  //Dataprint
  /*Serial.print("CO2[ppm]: ");
  Serial.println(CO2);
  Serial.print("CO [ppm]: ");
  Serial.println(CO);
  Serial.print("NO2 [ppm]: ");
  Serial.println(NO2);
  Serial.println("_______________");*/

  //LOG-V-DATA
  if (Serial.available()>0){
    char c = Serial.read();
    if(c = 83){
      Data = "$N$;Schadstoffmessung\r\n";
      Serial.print(Data);
      Data = "$C$;CO2[ppm];CO[ppm];NO2[ppm]\r\n";
      Serial.print(Data);
      XXX = 1;
      delay(1000);
    }
  }

  if(XXX == 1){
    Serial.print("$");
    Serial.print(CO2);
    Serial.print(";");
    Serial.print(CO);
    Serial.print(";");
    Serial.print(NO2);
    Serial.print("\r\n");
  }
  delay (2000);
}
