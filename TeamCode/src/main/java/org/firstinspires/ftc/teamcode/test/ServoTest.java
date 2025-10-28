package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "Motor");
        Servo servo1 = hardwareMap.get(Servo.class,"servo1");

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(0);

        }

        motor.setPower(0);
    }
}
