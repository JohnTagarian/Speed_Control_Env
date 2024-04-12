#include <Arduino.h>
#include "SpeedControl.h"


#define IN1 6
#define IN2 7
#define PWM 5

unsigned long timePlot;

Control control(IN1,IN2,PWM);

void setup() {
  Serial.begin(115200);
  control.init();
}

void loop() {
  // control.targetRPM = 100;

  while (Serial.available() > 0)
  {
    String inData = Serial.readStringUntil('\n');
    String data = inData.substring(inData.indexOf("=")+1,inData.indexOf(";"));

    if(inData[0] == 'p') control.kp = data.toFloat();
    else if(inData[0] == 'i') control.ki = data.toFloat();
    else if(inData[0] == 'd') control.kd = data.toFloat();
    else if(inData[0] == 's') control.setRPM(data.toFloat());
  }
  

  if(millis() - timePlot > 100){
    // timePlot = millis();
    control.showinfo();
    Serial << "KP : "<<control.kp << " , ";
    Serial << "KI : "<<control.ki << " , ";
    Serial << "KD : "<<control.kd << " , ";

    timePlot = millis();
  }

}
