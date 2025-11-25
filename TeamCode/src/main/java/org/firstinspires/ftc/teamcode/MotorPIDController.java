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
public class MotorPIDController {
    ElapsedTime timer=new ElapsedTime();
    public double prevTime=timer.seconds();
    public DcMotor controlled_motor;
    public double Kp, Ki, Kd;
    public double p,i,d;
    public double max_power;
    public double target=0;
    public double error=0,prevError=0;
    public MotorPIDController(DcMotor m, double Kp, double Ki, double Kd){
        controlled_motor=m;
        controlled_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controlled_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    //this function updates the position of the motor and returns the current error
    public double update(){
        prevError=error;
        error=controlled_motor.getCurrentPosition()-target;
        if(abs(error)>100){
            if(error>0)
                controlled_motor.setPower(-max_power);
            else
                controlled_motor.setPower(max_power);
            return error;
        }
        p=Kp*error;
        i+=Ki*error;
        d=(error-prevError)/(timer.seconds()-prevTime)*Kd;
        prevTime=timer.seconds();
        controlled_motor.setPower(-max(min(p+i+d,max_power),-max_power));
        return error;
    }

    public void setPIDArguments(double kp, double ki, double kd){
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
    }

    public double[] getPIDArguments(){
        return new double[]{Kp, Ki, Kd};
    }
}