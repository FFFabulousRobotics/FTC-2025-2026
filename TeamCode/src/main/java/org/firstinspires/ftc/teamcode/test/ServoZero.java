package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Zero Position Test")
public class ServoZero extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        waitForStart();
        Servo s=(Servo)(hardwareMap.get("servo"));
        while(opModeIsActive())
            s.setPosition(0);
    }
}