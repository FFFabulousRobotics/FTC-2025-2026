package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Simple Pathing Test", group = "Test")
public class SimplePathingTest extends LinearOpMode {

    private MecanumDrive drive;
    private SimplePathing pathing;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("状态", "正在初始化硬件...");
        telemetry.update();

        // 初始化硬件
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "BR");
        GoBildaPinpointDriver pp = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // 设置功率因数
        double[] powerFactors = {1.0, 1.0, -1.0, -1.0};

        // 初始化驱动
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear, pp, powerFactors);

        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始化简单的路径控制器
        pathing = new SimplePathing(drive,
                0.4,    // 旋转功率
                1,    // 移动功率
                2.0,    // 旋转容差(度)
                100.0    // 距离容差(mm)
        );

        // 设置减速距离
        pathing.setSlowDownDistance(200);

        // 目标位置
        double targetX = 500;    // mm
        double targetY = 500;    // mm
        double targetHeading = 45; // 度

        telemetry.addData("状态", "初始化完成");
        telemetry.addData("目标位置", "(%.1f, %.1f) mm", targetX, targetY);
        telemetry.addData("目标朝向", "%.1f°", targetHeading);
        telemetry.addData("控制说明",
                "A: X+10, B: X-10, X: Y+10, Y: Y-10\n" +
                        "Dpad左: 角度-5, Dpad右: 角度+5\n" +
                        "右肩键: 开始移动");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean isMoving = false;

        while (opModeIsActive()) {
            // 通过手柄调整目标位置
            if (gamepad1.a) {
                targetX += 10;
                telemetry.addData("调整", "X增加10: %.1f", targetX);
                sleep(200);
            }
            if (gamepad1.b) {
                targetX -= 10;
                telemetry.addData("调整", "X减少10: %.1f", targetX);
                sleep(200);
            }
            if (gamepad1.x) {
                targetY += 10;
                telemetry.addData("调整", "Y增加10: %.1f", targetY);
                sleep(200);
            }
            if (gamepad1.y) {
                targetY -= 10;
                telemetry.addData("调整", "Y减少10: %.1f", targetY);
                sleep(200);
            }
            if (gamepad1.dpad_left) {
                targetHeading -= 5;
                telemetry.addData("调整", "角度减少5: %.1f°", targetHeading);
                sleep(200);
            }
            if (gamepad1.dpad_right) {
                targetHeading += 5;
                telemetry.addData("调整", "角度增加5: %.1f°", targetHeading);
                sleep(200);
            }

            // 开始移动
            if (gamepad1.right_bumper) {
                pathing.setTarget(targetX, targetY, targetHeading);
                isMoving = true;
                runtime.reset();
                telemetry.addData("状态", "开始移动");
            }

            // 停止移动
            if (gamepad1.left_bumper) {
                pathing.stop();
                isMoving = false;
                telemetry.addData("状态", "手动停止");
            }

            // 更新路径控制器
            if (isMoving) {
                boolean reached = pathing.update();
                if (reached) {
                    isMoving = false;
                    telemetry.addData("状态", "✅ 到达目标");
                }
            }

            // 显示遥测信息
            sendTelemetry(targetX, targetY, targetHeading, isMoving);

            sleep(20);
        }

        // 程序结束，停止机器人
        drive.drive(0, 0, false, 0);
    }

    private void sendTelemetry(double targetX, double targetY, double targetHeading, boolean isMoving) {
        double currentX = -drive.getIMU().getPosX(DistanceUnit.MM);
        double currentY = -drive.getIMU().getPosY(DistanceUnit.MM);
        double currentHeading = drive.getIMU().getHeading(AngleUnit.DEGREES);
        double distance = Math.hypot(targetX - currentX, targetY - currentY);
        double headingError = pathing.getHeadingError();

        telemetry.addLine("===== 简单路径控制测试 =====");
        telemetry.addData("运行时间", "%.2f秒", runtime.seconds());
        telemetry.addData("移动状态", isMoving ? "⏳ 移动中" : "⏸️ 已停止");
        telemetry.addData("控制器状态", pathing.getCurrentState());

        telemetry.addLine();
        telemetry.addLine("===== 目标信息 =====");
        telemetry.addData("目标位置", "(%.1f, %.1f) mm", targetX, targetY);
        telemetry.addData("目标朝向", "%.1f°", targetHeading);

        telemetry.addLine();
        telemetry.addLine("===== 当前位置 =====");
        telemetry.addData("当前位置", "(%.1f, %.1f) mm", currentX, currentY);
        telemetry.addData("当前朝向", "%.1f°", currentHeading);

        telemetry.addLine();
        telemetry.addLine("===== 误差信息 =====");
        telemetry.addData("距离误差", "%.1f mm", distance);
        telemetry.addData("朝向误差", "%.1f°", headingError);
        telemetry.addData("是否到达目标", pathing.isAtTarget() ? "✅ 是" : "❌ 否");

        telemetry.addLine();
        telemetry.addLine("===== 控制说明 =====");
        telemetry.addData("开始移动", "右肩键");
        telemetry.addData("停止移动", "左肩键");
        telemetry.addData("调整目标", "A/B/X/Y/Dpad");

        telemetry.update();
    }
}