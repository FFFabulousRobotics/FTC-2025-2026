package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
//四轮（control hub）
//BR = 3
//BL = 2
//FL = 1
//FR = 0
//（expansion hub）
//LR = 0
//LL = 1
//IN = 3
//PRESS = 2
//PP位于control hub 的I2C3号口


@TeleOp(name = "ManualOpMode")
public class ManualOpMode extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        // 初始化电机
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "BR");
        DcMotor motor = (DcMotor) (hardwareMap.get("PRESS"));
        MotorPIDController pid = new MotorPIDController(motor, 0.1, 0.000005, 0.0005);
        DcMotor intake=(DcMotor)hardwareMap.get("IN");
        DcMotor left = hardwareMap.get(DcMotor.class, "LL");
        DcMotor right = hardwareMap.get(DcMotor.class, "LR");
        float lift=0;
        MotorPIDController left_pid=new MotorPIDController(left,0.1,0.00005,0.0005);
        MotorPIDController right_pid=new MotorPIDController(right,0.1,0.00005,0.0005);
        Servo s1 = (Servo) (hardwareMap.get("servo0"));
        Servo s2 = (Servo) (hardwareMap.get("servo1"));
        Servo s3 = (Servo) (hardwareMap.get("servo2"));
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        ElapsedTime time = new ElapsedTime();
        double servoTime = 0, motorTime = 0;
        // 功率因数校准数组 [左前, 右前, 左后, 右后]
        // 这些值需要根据实际测试进行调整
        // 例如，如果左前轮比其它轮子慢，可以将其因数设为1.1
        double[] powerFactors = {1.0, 1.0, -1.0, -1.0};

        // 创建驱动对象
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear, powerFactors);

        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // 从游戏手柄获取输入
            double gamepadX = gamepad1.left_stick_x;
            double gamepadY = -gamepad1.left_stick_y;  // 反转Y轴
            double rotation = gamepad1.right_stick_x;

            // 计算相对于机器人的方位角和功率
            double heading = Math.toDegrees(Math.atan2(gamepadX, gamepadY));
            double power = Math.sqrt(gamepadX * gamepadX + gamepadY * gamepadY);

            // 如果摇杆在中心附近，停止运动
            if (power < 0.1) {
                power = 0;
            }

            // 驱动机器人
            drive.drive(heading, power, rotation);

            // 调试信息
            telemetry.addData("Heading", "%.1f°", heading);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Rotation", "%.2f", rotation);
            telemetry.addData("Power Factors",
                    "LF:%.2f, RF:%.2f, LR:%.2f, RR:%.2f",
                    drive.getPowerFactor(0), drive.getPowerFactor(1),
                    drive.getPowerFactor(2), drive.getPowerFactor(3));
            telemetry.update();
            if (gamepad1.aWasPressed()) {
                pid.setTarget(-4000);
                motorTime = time.seconds();
            }
            if (time.seconds() - motorTime >= 1.5) {
                pid.setTarget(-500);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                s1.setPosition(0.5);
                servoTime = time.seconds();
            }
            if (gamepad1.dpadDownWasPressed()) {
                s2.setPosition(0.5);
                servoTime = time.seconds();
            }
            if (gamepad1.dpadRightWasPressed()) {
                s3.setPosition(0.5);
                servoTime = time.seconds();
            }
            if (gamepad1.dpadUpWasPressed()) {
                s1.setPosition(0.5);
                s2.setPosition(0.5);
                s3.setPosition(0.5);
                servoTime = time.seconds();
            }
            if (time.seconds() - servoTime >= 0.1) {
                s1.setPosition(0.6);
                s2.setPosition(0.6);
                s3.setPosition(0.4);
            }
            pid.update();
            if (gamepad1.xWasPressed()) {
                intake.setPower(1);
//                intake.setPower(0);
            }
            if (gamepad1.yWasPressed()){
                intake.setPower(0);
            }
            if(gamepad1.right_trigger>0)
            {
                left_pid.setTarget(-5750);
                right_pid.setTarget(-5922);
            }
            if(gamepad1.left_trigger>0)
            {
                left_pid.setTarget(-10);
                right_pid.setTarget(-10);
            }
            left_pid.update();
            right_pid.update();
        }
        pid.setTarget(0);
        while (abs(pid.update()) > 5) ;
        motor.setPower(0);
        drive.stop();

    }
}