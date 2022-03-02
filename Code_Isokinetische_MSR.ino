#include <PID_v1.h>

//Hardware(Definition der Anschlüsse die Physisch am Arduino sind)

int Pumpe             = 3;                    //Hier wird der Pin für den Anschluss der Pumpe definiert
int Sensor_Probenrohr = A1;                   //Hier wird der Pin für den Anschluss des Sensors im Probenrohr definiert
int Sensor_Abgas      = A2;                   //Hier wird der Pin für den Anschluss des Sensors im Abgas definiert

//Variables(Definition/Erstellen der Variablen in Welchen werte Gespeichert werden können)  

  //PID                                         Hier werden die Werte für die PID-Kontrolle Definiert bzw. Erstellt.

  double Input;                               //Das ist der Wert welcher von den Sensoren Erstellt wird und die Differenz
                                              //zwischen Probenrohr und Abgasgeschwinigkeit Angibt.
  double Output;                              //Das ist der Wert der der Arduino an die Pumpe schickt nach der Berechnung.
  double Setpoint = 0;                        //Hier wird der Punkt definiert wo der Input hin soll bei uns Differenz von 0.
  
  double Kp = 0.2;                            //Dies sind die Variablen die Angeben wie der PID Algorhytmus auf veränderung reagiert.      
  double Ki = 0.2;
  double Kd = 0;

  //Sensors                                     Hier werden die Variablen zum Rechnen und Speichern der Sensordaten erstellt.
  float Sensor_Val_Probenrohr;
  float Sensor_Val_Abgas;
  float Sensor_Val_Probenrohr_Total;
  float Sensor_Val_Abgas_Total;
  float Sensor_Val_Probenrohr_AVG;
  float Sensor_Val_Abgas_AVG;
  int readIndex_Pumpe = 0;
  int readIndex_Rohr = 0;

  //Pumpe
  int Pumpspeed;                                //Hier wird der Pumpspeed erstellt welche in PPM der Pumpe sagt wie schnell sie sein soll.

  //Smoothing                                   //Hier wird ein Smoothing Erstellt um Ausreiser zu devalurisierne
  int Samplesmade;
  int Samples = 50;                             //Hier kann definiert werden nach wie vielen Messungen ein Durchschnittswert genommen werden soll.
  int Time4S = 50;                              //Hier kann definiert werden wie viel Zeit zwischen einer Messung in ms vergeht.

//Commands
PID Pumpsteuerung(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE); 
//Hier wird die PID berechnung erstellt somit können auch mehrere unabhänige PID berechnungne statfinden falls mehr benötigt werden.

void setup() {
  Serial.begin(9600);                          //Hier wird die Bautrate festgelegt das heißt wie schnell mit dem PC kommuniziert wird.

  //PID
  PID Pumpsteuerung(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

  //Sensors                                   //Hier werden die Pins auf INPUT oder OUTPUT gestellt 
  pinMode(Sensor_Probenrohr, INPUT);  
  pinMode(Sensor_Abgas, INPUT);

  //Pumpe
  pinMode(Pumpe, OUTPUT);


}

void loop() {

//Sensorread                                  //Hier werden die Sensoren ausgelesen und gesmoothed 
  while(Samplesmade < Samples){
    Sensor_Val_Probenrohr = analogRead(Sensor_Probenrohr);
    Sensor_Val_Abgas = analogRead(Sensor_Abgas);

    Sensor_Val_Probenrohr_Total = Sensor_Val_Probenrohr_Total + Sensor_Val_Probenrohr;
    Sensor_Val_Abgas_Total = Sensor_Val_Abgas_Total + Sensor_Val_Abgas;

    Samplesmade ++;
    delay(Time4S);
    
  }

 Sensor_Val_Probenrohr_AVG = Sensor_Val_Probenrohr_Total / Samples;
 Sensor_Val_Abgas_AVG = Sensor_Val_Abgas_Total / Samples;
 
 Samplesmade = 0;

 
//PID n Pump
  PID1.SetMode(AUTOMATIC);
  Input = Sensor_Val_Abgas_AVG - Sensor_Val_Probenrohr_AVG;
  PID1.Compute();

  //Hier wird jetzt geprüft ob der Output über der minderstgeschwindigkeit der Pumpe ist.
  if(Output >= 150){
    analogWrite(Pumpe, Output);
  }
  else{
    analogWrite(Pumpe, 150);
    Output = 150;
  }


//Überwachung           //Hier können zur überwachung am PC die Ausgegebenen werte eingesehen werden.

  Serial.println("SensorPumpe   SensorRohr   PID-OUTPUT");
  Serial.print(Sensor_Val_Probenrohr_AVG);
  Serial.print("           ");
  Serial.print(Sensor_Val_Abgas_AVG);
  Serial.print("           ");
  Serial.println(Output);

//AVG Clear

Sensor_Val_Probenrohr_Total = 0;
Sensor_Val_Abgas_Total = 0;
}
