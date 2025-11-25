package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;



@TeleOp
public class MotorTest extends LinearOpMode {
    RobotHardware hardware;

    public void runOpMode()
    {
        DcMotor motor;
        motor = hardwareMap.get(DcMotor.class, "Motor1");
        while (true)
            motor.setPower(1);
    }
}


