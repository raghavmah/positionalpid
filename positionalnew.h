#include <PID_v1.h>
#include <Motor.h>

//Create Encoder Object from its pin(En1, En2 , DirectionalOffset)
//Create Motor Object (Pwm pin, Dir pin1, Dir pin2 (Optional))
#define pulseMode 0

class positionalnew{
public:
	Motor *mtr = new Motor();

	int mode = pulseMode;
    double Setpoint = 0, Input, Output;
    int targetPulse = 0;
    double aggKp=0.03, aggKi=0, aggKd=0.00;
    
    double softKp=0.01, softKi=0, softKd=0.00;
    
    int softThreshold = 0;
    long diff  = 0;
    bool enable = true;
    int speed = 0;
    PID *myPID = new PID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
    int sampleTime = 0;

    positionalnew(){
        myPID->SetMode(AUTOMATIC);
        myPID->SetSampleTime(this->sampleTime);
    }
    positionalnew(Motor *mtr){
        myPID->SetMode(AUTOMATIC);
        myPID->SetSampleTime(this->sampleTime);
        this->mtr = mtr;
    }
    void setThreshold(int targetThreshold){
        softThreshold = targetThreshold;
    }
    

    void setPulse(int targetPulse){
        
        this->targetPulse = targetPulse;
        mode = pulseMode;
    }
    void setOutputLimits(int min,int max){
        myPID->SetOutputLimits(min,max);
    }
    void setAggTunings(double Kp, double Ki, double Kd){
        this->aggKp = Kp;
        this->aggKi = Ki;
        this->aggKd = Kd;
        myPID->SetTunings(Kp,Ki,Kd);
    }
    void setSoftTunings(double Kp, double Ki, double Kd){
        this->softKp = Kp;
        this->softKi = Ki;
        this->softKd = Kd;
        myPID->SetTunings(Kp,Ki,Kd);
    }



    void compute(){
        if(enable){
           
            diff = mtr->getReadings() - targetPulse;
            Input = diff;
            Serial.print(diff);
            if(abs(Input)<softThreshold){
                this->setSoftTunings(softKp,softKi,softKd);
            }
            else{
                this->setAggTunings(aggKp,aggKi,aggKd);
            }
            myPID->Compute();
            speed = diff<0?(Output):(Output * -1);
            Serial.print("Speed = ");
            Serial.print(speed);
            mtr->setPWM(speed);
        }
    }

   

};