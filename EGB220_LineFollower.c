#define F_CPU 16000000UL

#include <avr/io.h>
#include "Motors.h"
#include "Sensors.h"
#include "math.h"


// Set Colour Threshold
#define UTHRESH 20
#define LTHRESH 250 //1 lap gave fullspeed of 27, speedlimit of 50, kp of 3 or 4 or 6, kd of 3 or 4
int FULL_SPEED = 50;
#define SPEEDLIMIT 45

// Declare global variables
int motorSpeeds[2] = {0,0};
uint8_t SensorBool[6];
uint8_t SensorVal[6];
int lastError = 0;
int corner;
int zone;

//sensorreading fuction
double getPosition()
{
    uint8_t SensorVal[6];
    double position = 0;
    double subpos;
    

    //iterate through sensors storing their values in the arrays, as well as if they have a poitive reading
    int i;
    for (i = 0; i < 6; i++)
    {
        SensorVal[i] = SensorValue(i+1);
        if (SensorVal[i] < UTHRESH)
        {
            subpos = 4*pow(i,2)-20*i + 40;
            if(i>=3)
            {
                subpos = subpos * -1;
            }
            position += subpos; //quadratic for increased weighting at edges
        }
        if (SensorVal[i] > LTHRESH)
        {
            position += 0; //i know this doesnt do anything, its just for reading logic
        }
        else
        {
            position += ((255-SensorVal[i])/255)*(2.5-i); //might need to adjust the /255 due to the above band pass filter 255 might change to (LTHRESH-UTHRES)
        }
        
    }
    return position;
}

// PID control to set motor speeds accordingly
void PID(double kp, double kd, double PrevError, int SlowArea, int Corner)
{
    int LeftMotor;
    int RightMotor;
    double PIDoutput;
    double position;
    double error;
    double derivitive;
    int CornerToggle;

    position = getPosition(); 
    error = position;  
        
    derivitive = error - lastError;

    PIDoutput = (kp*error) + (kd*derivitive);
    lastError = error;

    RightMotor = FULL_SPEED - (int)PIDoutput;
    LeftMotor = FULL_SPEED + (int)PIDoutput;

    // Set motor speeds accordingly
    motorSpeeds[0] = LeftMotor; // Left is 0
    motorSpeeds[1] = RightMotor; // Right is 1
    
    double diff = abs(RightMotor-LeftMotor);

    
    
}

// Main Loop
int main(void)
{
    timer0_init();
    timer1_init();
    ADC_init();
    DDRE |= (1<<6);//LED1
    DDRB |= (1<<0)|(1<<1)|(1<<2);//LED2,3,4
    
    motorSpeeds[0] = 0; 
    motorSpeeds[1] = 0;
    
    int RPrevSpeed = FULL_SPEED;
    int LPrevSpeed = FULL_SPEED;
    
    setMotorSpeeds(motorSpeeds);
    double PrevError;

    while(1)
    {
        
        // Right sensor: SensorValue(7)
        if(SensorValue(8) < UTHRESH)
        {
            PORTE |= (1<<6);//LED1
            PORTB |= (1<<0)|(1<<1)|(1<<2);//LED2,3,4
        }

        else
        {
            PORTE &= ~(1<<6);//LED1
            PORTB &= ~(1<<0);
            PORTB &= ~(1<<1);
            PORTB &= ~(1<<2);//LED2,3,4
        }
        

        PID(1.5,20, PrevError, 0, 0);
        setMotorSpeeds(motorSpeeds);
    }
    return 0;
}
