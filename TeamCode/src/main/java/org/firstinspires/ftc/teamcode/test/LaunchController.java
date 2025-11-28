package org.firstinspires.ftc.teamcode.test;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MotorPIDController;
public class LaunchController{
    DcMotor motor;
    MotorPIDController pidController;
    public boolean waiting=true;
    public int state=0;//0-rotating -540 units,1-rotating until stuck,2-rotating back to -180 units
    double error = 0.0,last_error;
    ElapsedTime time = new ElapsedTime();
    double prevTime=0;
    public LaunchController(DcMotor motor,MotorPIDController pidController) {
        pidController.setMaxPower(1);
        this.motor=motor;
        this.pidController=pidController;
    }
    public void loadLauncher() {
        waiting=false;
        state=0;
        pidController.setTarget(-1660);
    }
    public double update(double thres)
    {
        if(!waiting) {
            if (state==0) {
                if (abs(pidController.update()) < 10)
                {
                    prevTime=time.seconds();
                    state = 1;
                    return 0;
                }
            }
            else if (state==1) {
                motor.setPower(-1);
                last_error = error;
                error = (double) motor.getCurrentPosition();
                if (abs(error - last_error)/(time.seconds()-prevTime) < thres) {
                    state = 2;
                    pidController.setTarget(100);
                }
                double t=abs(error - last_error)/(time.seconds()-prevTime);
                prevTime=time.seconds();
                return t;
            }
            else if(state==2){
                if(abs(pidController.update()) < 5) {
                    waiting=true;
                    state=0;
                    motor.setPower(0);
                }
                return 0;
            }
        }
        return 0;
    }
}