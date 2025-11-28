package org.firstinspires.ftc.teamcode.test;
//import static java.lang.Math.abs;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MotorPIDController;
//import org.firstinspires.ftc.teamcode.test.LaunchController;

@TeleOp(name="Launch motor test")
public class LaunchTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motor=(DcMotor)(hardwareMap.get("PRESS"));
        MotorPIDController pid=new MotorPIDController(motor,0.1,0.00005,0.0005);
        Servo s1=(Servo)(hardwareMap.get("servo0"));
        Servo s2=(Servo)(hardwareMap.get("servo1"));
        Servo s3=(Servo)(hardwareMap.get("servo2"));
        s1.setPosition(0.6);
        s2.setPosition(0.6);
        s3.setPosition(0.4);
        ElapsedTime time=new ElapsedTime();
        double servoTime=0,motorTime=0;
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.aWasPressed()) {
                pid.setTarget(-4000);
                motorTime=time.seconds();
            }
            if(gamepad1.bWasPressed()||time.seconds()-motorTime>=1.5)
                pid.setTarget(-500);
            if(gamepad1.dpadLeftWasPressed()) {
                s1.setPosition(0.5);
                servoTime=time.seconds();
            }
            if(gamepad1.dpadDownWasPressed()) {
                s2.setPosition(0.5);
                servoTime=time.seconds();
            }
            if(gamepad1.dpadRightWasPressed()) {
                s3.setPosition(0.5);
                servoTime=time.seconds();
            }
            if(gamepad1.dpadUpWasPressed()) {
                s1.setPosition(0.5);
                s2.setPosition(0.5);
                s3.setPosition(0.5);
                servoTime=time.seconds();
            }
            if(time.seconds()-servoTime>=0.1) {
                s1.setPosition(0.6);
                s2.setPosition(0.6);
                s3.setPosition(0.4);
            }
            pid.update();
        }
        pid.setTarget(0);
        while(abs(pid.update())>5);
        motor.setPower(0);
    }
}
