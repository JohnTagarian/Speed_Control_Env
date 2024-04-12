#include <Arduino.h>
#include "SpeedControl.h"
#include <Streaming.h>


IntervalTimer speed_control_timer;


//static
volatile long Control::encCnt = 0;
volatile long Control::prevTime =0;
volatile float Control::velocity = 0.0;
float Control::rpm = 0.0;
int Control::targetRPM = 0;
float Control::eint = 0.0;
long Control::prevTimeI =0.0;
float Control::prevError = 0.0;
uint8_t Control::_DIR1 = 0;
uint8_t Control::_DIR2 = 0;
uint8_t Control::_PWM = 0;

// 100 - 500;
// float Control::kp = 4.3;
// float Control::ki = 2.4;
// float Control::kd = 0.16;

float Control::kp = 4.3;
float Control::ki = 3.1;
float Control::kd = 0.12;

float Control::rpmFilt = 0.0;
float Control::rpmPrev = 0.0;


Control::Control(uint8_t DIR1,uint8_t DIR2,uint8_t PWM){
    _DIR1 = DIR1;
    _DIR2 = DIR2;
    _PWM = PWM;
    
    analogWriteResolution(10);
    analogWriteFrequency(PWM,1000);
    
    pinMode(_DIR1 ,OUTPUT);
    pinMode(_DIR2, OUTPUT);
    pinMode(_PWM,OUTPUT);
}

void Control::init(){
    pinMode(_ENCA,INPUT);
    pinMode(_ENCB,INPUT);
    attachInterrupt(digitalPinToInterrupt(_ENCA),readenc,RISING);
    speed_control_timer.begin(compute_pid,10000);
}

void Control::readenc(){
    int increment = (digitalRead(_ENCB) == 1)? 1:-1;
    encCnt += increment;


    long currentTime = micros();

    float deltaTime = ((float)(currentTime - prevTime)) / 1.0e6;
    velocity = 1/deltaTime;
    prevTime = currentTime;
}

void Control::setMotor(int16_t pwr){
    digitalWrite(_DIR1,pwr > 0 ? HIGH:LOW);
    digitalWrite(_DIR2,pwr > 0 ? LOW:HIGH);
    analogWrite(_PWM,pwr == 0 ? 0:abs(pwr));

}

void Control::showinfo(){

    Serial.print(">filt:");
    Serial.println(rpmFilt);

    Serial.print(">Target:");
    Serial.println(targetRPM);

    Serial.print(">Eint:");
    Serial.println(eint);
    
    // Serial.print("|");
}
void Control::setRPM(int RPM){
    targetRPM = RPM;

}

void Control::compute_pid(){
    rpm = (velocity/200.0)*60.0;  //RPM
    rpmFilt = 0.854*rpmFilt + 0.0728*rpm + 0.0728*rpmPrev;
    rpmPrev = rpm;

    float error = abs(targetRPM) - rpmFilt;

    eint += error*0.01f;
    eint = constrain(eint,-8000.0,8000.0);
    if(targetRPM == 0) eint = 0.0;

    float diffError = (error - prevError)/0.01f;
    prevError = error;
    if(targetRPM == 0) diffError = 0.0;
    
    float u = kp*error + ki*eint + kd*diffError;
    u = constrain(u,0,1023);
    if(targetRPM < 0) u =-u;
    setMotor(u);
    
    // Serial.print(">U:");
    // Serial.println(u);
    // Serial << "Target : " << targetRPM << " ";
    // Serial << "Error :"<< error << " ";
    // Serial << "RPM : "<< rpm << endl;

}