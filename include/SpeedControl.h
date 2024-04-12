#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H
#include <Streaming.h>

#define _ENCA 23
#define _ENCB 22



class Control
{
    private:
        static uint8_t _DIR1;
        static uint8_t _DIR2;
        static uint8_t _PWM;

        // float kp;
        // float ki;
        // float kd;
        static float rpm;
        static float rpmPrev;
        static float eint;
        static float prevError;
        static long prevTimeI;
        static float rpmFilt;
        static int targetRPM;

        
    public:
        
        static volatile long encCnt;
        static volatile long prevTime;
        static volatile float velocity;

        
        static float kp;
        static float ki;
        static float kd;
        
        

        Control(uint8_t DIR1,uint8_t DIR2,uint8_t PWM );
        static void init();
        static void readenc();
        static void compute_pid();

        static void setMotor(int16_t);
        void showinfo();
        void setRPM(int);
        



};




#endif