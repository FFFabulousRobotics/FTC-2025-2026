package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Trigger Test")
public class ServoZero extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        waitForStart();
        Servo s1=(Servo)(hardwareMap.get("servo0"));
        Servo s2=(Servo)(hardwareMap.get("servo1"));
        Servo s3=(Servo)(hardwareMap.get("servo2"));
        while(opModeIsActive()){
            while(!gamepad1.a);
            s1.setPosition(0.6);
            s2.setPosition(0.6);
            s3.setPosition(0.4);
            while(!gamepad1.b);
            s1.setPosition(0.5);
            s2.setPosition(0.5);
            s3.setPosition(0.5);
        }
    }
}
