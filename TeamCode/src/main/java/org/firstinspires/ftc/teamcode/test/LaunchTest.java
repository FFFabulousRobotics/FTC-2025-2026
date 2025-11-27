package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MotorPIDController;
import org.firstinspires.ftc.teamcode.test.LaunchController;

@TeleOp(name="Launch motor test")
public class LaunchTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motor=(DcMotor)(hardwareMap.get("motor1"));
        MotorPIDController pid=new MotorPIDController(motor,0.1,0.00005,0.0005);
        LaunchController lc=new LaunchController(motor,pid);
        waitForStart();
        Gamepad prevGamepad1 = new Gamepad();
        double thres=100;
        while(opModeIsActive()) {
            if(gamepad1.a&&!prevGamepad1.a)
                lc.loadLauncher();
            double d=lc.update(thres);
            if(gamepad1.left_bumper&&!prevGamepad1.left_bumper)
                thres-=thres/50;
            if(gamepad1.right_bumper&&!prevGamepad1.right_bumper)
                thres+=thres/49;
            telemetry.addData("stage",lc.state);
            telemetry.addData("dx/dt",d);
            telemetry.addData("thres",thres);
            telemetry.update();
            prevGamepad1.copy(gamepad1);
        }
    }
}
