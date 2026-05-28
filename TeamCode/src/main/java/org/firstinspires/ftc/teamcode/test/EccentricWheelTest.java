package org.firstinspires.ftc.teamcode.test;

import static java.lang.Math.abs;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MotorPIDController;

@TeleOp
public class EccentricWheelTest extends OpMode {
    private DcMotor motor;
    private MotorPIDController pid;
    private double Target = 0;

    @Override
    public void init() {
        motor = (DcMotor) (hardwareMap.get("PRESS"));
        pid = new MotorPIDController(motor, 0.01, 0.0000001, 0.0005);

        pid.setTarget(Target);
        pid.setMaxPower(0.6);
    }

    @Override
    public void loop() {
        if(gamepad1.leftBumperWasPressed()) {
            Target -= 10;
            pid.setTarget(Target);
        }

        if(gamepad1.rightBumperWasPressed()) {
            Target += 10;
            pid.setTarget(Target);
        }
        if(gamepad1.aWasPressed()) motor.setPower(0.3);
        if(gamepad1.bWasPressed()) motor.setPower(-0.3);
        if(gamepad1.xWasPressed()) ;

        if(abs(motor.getCurrentPosition() - Target) >= 5) {
            pid.update();
        }

        else {
            motor.setPower(0);
        }

        telemetry.addData("Target", pid.target);
        telemetry.addData("Current", motor.getCurrentPosition());
        telemetry.addData("Error", motor.getCurrentPosition() - Target);
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("P", pid.p);
        telemetry.addData("I", pid.i);
        telemetry.addData("D", pid.d);
        telemetry.addData("Target Increment", Target);
        telemetry.update();
    }




}
