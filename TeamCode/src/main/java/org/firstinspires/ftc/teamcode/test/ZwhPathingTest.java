package org.firstinspires.ftc.teamcode.test;

import static java.lang.Math.random;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.test.ZwhPathing;

@Autonomous(name = "ZwhPathing Test", group = "Test")
public class ZwhPathingTest extends LinearOpMode {

    private MecanumDrive drive;
    private ZwhPathing pathing;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("状态", "正在初始化硬件...");
        telemetry.update();

        // 从硬件映射中获取电机和IMU
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "BR");
        GoBildaPinpointDriver pp = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // 设置功率因数（可以根据实际情况调整）
        double[] powerFactors = {1.0, 1.0, -1.0, -1.0}; // 左前, 右前, 左后, 右后

        // 初始化驱动
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear, pp, powerFactors);

        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 或者使用自定义PID参数（取消注释以下代码使用自定义参数）
        double[] distancePIDParams = {5e-2, 8e-2, 0.01};
        double[] anglePIDParams = {8e-2, 4e-2, 0.01};
        pathing = new ZwhPathing(drive, 0,
                distancePIDParams, anglePIDParams,
                1,  // 最大功率
                2.0,  // 旋转容差（度）
                10.0  // 距离容差（mm）
        );
        double targetX = 663,targetY = 709,targetHeading = 36;
        telemetry.addData("状态", "初始化完成，等待开始...");
        telemetry.addData("目标位置", "(%.1f, %.1f) mm", targetX, targetY);
        telemetry.addData("目标朝向", "%.1f°", targetHeading);
        telemetry.addData("起始位置", "确保机器人在场地坐标系原点");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // 设置目标位置和朝向
        pathing.setTarget(targetX, targetY, targetHeading);
        telemetry.addData("状态", "开始移动到目标...");
        telemetry.update();
        boolean targetReached = false;
        // 主控制循环
        while (opModeIsActive()) {
            telemetry.addData("dx,dy","%.1f,%.1f",pathing.dx,pathing.dy);
            telemetry.addData("moveHeading","%.1f",pathing.moveHeading);
            if(gamepad1.yWasPressed()) {
                targetX+=10;
                pathing.setTarget(targetX,targetY,targetHeading);
                runtime.reset();
            }
            if(gamepad1.aWasPressed()) {
                targetX-=10;
                pathing.setTarget(targetX,targetY,targetHeading);
                runtime.reset();
            }
            if(gamepad1.xWasPressed()) {
                targetY+=10;
                pathing.setTarget(targetX,targetY,targetHeading);
                runtime.reset();
            }
            if(gamepad1.bWasPressed()) {
                targetY-=10;
                pathing.setTarget(targetX,targetY,targetHeading);
                runtime.reset();
            }
            if(gamepad1.dpadLeftWasPressed()) {
                targetHeading-=5;
                pathing.setTarget(targetX,targetY,targetHeading);
                runtime.reset();
            }
            if(gamepad1.dpadRightWasPressed()) {
                targetHeading+=5;
                pathing.setTarget(targetX,targetY,targetHeading);
                runtime.reset();
            }
            if(gamepad1.leftBumperWasPressed())
            {
                targetX=50;
                targetY=-50;
                targetHeading=180;
                pathing.setTarget(targetX,targetY,targetHeading);
                targetReached=false;
            }
            if(gamepad1.rightBumperWasPressed())
            {
                pathing.setTarget(targetX,targetY,targetHeading);
                targetReached=false;
            }
            // 更新路径控制器
            if(!targetReached)
                targetReached = pathing.update();
            sendTelemetry(targetX,targetY,targetHeading);
            // 小延迟以避免循环过快
            sleep(2);
        }

        if (targetReached) {
            telemetry.addData("状态", "✅ 成功到达目标！");
            telemetry.addData("总用时", "%.2f 秒", runtime.seconds());
        } else {
            telemetry.addData("状态", "❌ 未能到达目标");
            telemetry.addData("当前距离目标", "%.1f mm", pathing.getDistanceToTarget());
            telemetry.addData("当前朝向误差", "%.1f°", pathing.getHeadingError());
        }
        // 停止机器人
        drive.drive(0, 0, false, 0);
        telemetry.update();
        // 保持最后状态3秒
        sleep(3000);
    }

    // 发送遥测数据
    private void sendTelemetry(double tx,double ty,double th) {
        double currentDistance = pathing.getDistanceToTarget();
        double headingError = pathing.getHeadingError();

        telemetry.addLine("===== 路径追踪状态 =====");
        telemetry.addData("目标","(%.2f,%.2f) %.1f°",tx,ty,th);
        telemetry.addData("运行时间", "%.2f 秒", runtime.seconds());
        telemetry.addData("距离目标", "%.1f mm", currentDistance);
        telemetry.addData("是否到达目标", pathing.isAtTarget() ? "✅ 是" : "⏳ 否");

        // 添加进度条显示
        double maxDistance = Math.hypot(tx,ty);
        double progress = Math.max(0, 100 * (1 - currentDistance / maxDistance));
        telemetry.addData("进度", "%.1f%%", progress);

        // 可视化进度条（字符进度条）
        int barLength = 20;
        int filledLength = (int) (barLength * progress / 100);
        StringBuilder progressBar = new StringBuilder("[");
        for (int i = 0; i < barLength; i++) {
            if (i < filledLength) {
                progressBar.append("=");
            } else {
                progressBar.append(" ");
            }
        }
        progressBar.append("]");
        telemetry.addData("进度条", progressBar.toString());

        // 添加建议
        if (currentDistance > 200) {
            telemetry.addData("建议", "正在快速接近目标...");
        } else if (currentDistance > 50) {
            telemetry.addData("建议", "正在减速接近...");
        } else if (Math.abs(headingError) > 5) {
            telemetry.addData("建议", "正在调整朝向...");
        } else {
            telemetry.addData("建议", "已接近目标...");
        }

        // 显示机器人的IMU信息
        telemetry.addLine();
        telemetry.addLine("===== IMU 信息 =====");
        telemetry.addData("当前位置X", "%.1f mm", drive.getIMU().getPosX(DistanceUnit.MM));
        telemetry.addData("当前位置Y", "%.1f mm", drive.getIMU().getPosY(DistanceUnit.MM));
        telemetry.addData("当前朝向", "%.1f°", drive.getIMU().getHeading(AngleUnit.DEGREES));

        telemetry.update();
    }
}