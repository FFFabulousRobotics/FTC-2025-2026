package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Intake motor rotation test")
public class IntakeTest extends LinearOpMode{
    @Override
    public void runOpMode() {
        DcMotor intake=(DcMotor)hardwareMap.get("IN");
        waitForStart();
        while(opModeIsActive())
            intake.setPower(1);
        intake.setPower(0);
    }
}
