package org.firstinspires.ftc.teamcode.test;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MotorPIDController;

@TeleOp
public class ShootTest extends OpMode {
    private DcMotor motor;
    private MotorPIDController pid;
    private Servo s1, s2, s3;
    ElapsedTime time = new ElapsedTime();
    double servoTime = 0, target = -5000;

    @Override
    public void init() {
        motor = (DcMotor) (hardwareMap.get("PRESS"));
        pid = new MotorPIDController(motor, 0.01, 0.0000001, 0.0005);
        s1 = (Servo) (hardwareMap.get("servo0"));
        s2 = (Servo) (hardwareMap.get("servo1"));
        s3 = (Servo) (hardwareMap.get("servo2"));

        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);

        pid.setTarget(1200);
        double pidtime = time.seconds();
        while(abs(pid.update()) > 5 && time.seconds() - pidtime <= 0.5) ;
        pid.setTarget(target);
    }

    @Override
    public void loop() {
        if(gamepad1.aWasPressed()) pid.setTarget(target);
        if(gamepad1.bWasPressed()) pid.setTarget(1200);
        if(gamepad1.leftBumperWasPressed()) target -= 10;
        if(gamepad1.rightBumperWasPressed()) target +=10;

        double CurrentError = pid.update();

        if(abs(CurrentError) <= 1800 && pid.getTarget() == target) pid.setTarget(1200);
        if(abs(CurrentError) <= 5 && pid.getTarget() == 1200) {
            if(gamepad1.dpadDownWasPressed()) {
                s1.setPosition(0.5);
                s2.setPosition(0.5);
                s3.setPosition(0.5);
                servoTime = time.seconds();
            }

            if(time.seconds() - servoTime >= 0.1) {
                s1.setPosition(0.4);
                s2.setPosition(0.6);
                s3.setPosition(0.6);
            }
        }

        telemetry.addData("Target", pid.getTarget());
        telemetry.addData("Current", motor.getCurrentPosition());
        telemetry.addData("Error", CurrentError);
        telemetry.update();
    }
}
