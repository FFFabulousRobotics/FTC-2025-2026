package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//this class is for the control of DC motors with the PID algorithm
//upon construction, it accepts a motor and three coefficients, Kp, Ki and Kd
//after a target position is set, use update() to automatically control the motor
//so that it rotates to the specified position
//note that if the distance is far enough(>100 units), it will simply rotate with max power(set value)
//until it is close enough to the target
public class GenericPIDController {
    ElapsedTime timer=new ElapsedTime();
    public double prevTime=timer.seconds();
    public double Kp, Ki, Kd;
    public double p,i,d;
    public double max_power=1;
    public double target=0;
    public double error=0,prevError=0;
    public GenericPIDController(double Kp, double Ki, double Kd){
        this.Kp=Kp;
        this.Ki=Ki;
        this.Kd=Kd;
    }
    public void setTarget(double target)
    {
        this.target=target;
    }
    public void setMaxPower(double maxPower){
        max_power=maxPower;
    }
    public void reset(){
        timer.reset();
        p=i=d=0;
        target=0;
    }
    public void setPidCoefficients(double Kp,double Ki,double Kd){
        this.Kp=Kp;
        this.Ki=Ki;
        this.Kd=Kd;
    }
    //this function uses current error to calculate how much power should be used
    public double update(double input){
        prevError=error;
        error=input-target;
        if(abs(error)>100){
            prevTime=timer.seconds();
            if(error>0)
                return -max_power;
            else
                return max_power;
        }
        p=Kp*error;
        i+=Ki*error*(timer.seconds()-prevTime);
        d=(error-prevError)/(timer.seconds()-prevTime)*Kd;
        prevTime=timer.seconds();
        return max(min(-p-i-d,max_power),-max_power);
    }
}