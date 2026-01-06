package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import static java.lang.Math.*;

public class SimplePathing {
    private final MecanumDrive drive;

    // 目标状态
    private double targetX, targetY;
    private double targetHeading;

    // 控制参数
    private final double rotationPower;  // 旋转功率
    private final double movePower;      // 移动功率
    private final double rotationTolerance;  // 旋转容差(度)
    private final double distanceTolerance;  // 距离容差(mm)

    // 状态跟踪
    private enum State {
        ROTATING,   // 旋转到目标朝向
        MOVING,     // 移动到目标位置
        COMPLETE    // 完成
    }
    private State currentState = State.COMPLETE;

    // 简单控制参数
    private double slowDownDistance = 300;  // 开始减速的距离(mm)

    // 调试信息
    private double currentHeadingError = 0;
    private double currentDistance = 0;

    public SimplePathing(MecanumDrive drive,
                         double rotationPower,
                         double movePower,
                         double rotationTolerance,
                         double distanceTolerance) {
        this.drive = drive;
        this.rotationPower = Math.abs(rotationPower);
        this.movePower = Math.abs(movePower);
        this.rotationTolerance = rotationTolerance;
        this.distanceTolerance = distanceTolerance;
    }

    // 设置目标位置和朝向
    public void setTarget(double x, double y, double headingDegrees) {
        this.targetX = x;
        this.targetY = y;
        this.targetHeading = headingDegrees;
        this.currentState = State.ROTATING;
    }

    // 设置只移动（不改变朝向）
    public void setTarget(double x, double y) {
        setTarget(x, y, drive.getIMU().getHeading(AngleUnit.DEGREES));
    }

    // 角度归一化到[-180, 180)
    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    // 获取当前状态
    public State getCurrentState() {
        return currentState;
    }

    // 获取到目标的距离
    public double getDistanceToTarget() {
        double currentX = -drive.getIMU().getPosX(DistanceUnit.MM);
        double currentY = -drive.getIMU().getPosY(DistanceUnit.MM);
        return hypot(targetX - currentX, targetY - currentY);
    }

    // 获取朝向误差
    public double getHeadingError() {
        double currentHeading = drive.getIMU().getHeading(AngleUnit.DEGREES);
        return normalizeAngle(targetHeading + currentHeading);
    }

    // 检查是否到达目标
    public boolean isAtTarget() {
        currentDistance = getDistanceToTarget();
        currentHeadingError = Math.abs(getHeadingError());
        return currentDistance < distanceTolerance && currentHeadingError < rotationTolerance;
    }

    // 简单的旋转控制 - 使用固定功率，接近目标时停止
    private void rotateToHeading() {
        double headingError = getHeadingError();
        currentHeadingError = Math.abs(headingError);

        // 如果误差很小，旋转完成
        if (currentHeadingError < rotationTolerance) {
            drive.drive(0, 0, false, 0);
            currentState = State.MOVING;
            return;
        }

        // 使用固定功率旋转，方向由误差正负决定
        double power = Math.copySign(rotationPower, headingError);
        drive.drive(0, 0, false, power);
    }

    // 简单的移动控制 - 匀速运动，接近目标时减速
    private void moveToPosition() {
        double currentX = -drive.getIMU().getPosX(DistanceUnit.MM);
        double currentY = -drive.getIMU().getPosY(DistanceUnit.MM);

        // 计算到目标的向量
        double dx = targetX - currentX;
        double dy = targetY - currentY;
        currentDistance = hypot(dy, dx);

        // 如果到达目标，完成
        if (currentDistance < distanceTolerance) {
            drive.drive(0, 0, false, 0);
            currentState = State.COMPLETE;
            return;
        }

        // 计算移动方向（绝对角度）
        double moveAngle = toDegrees(atan2(dy, dx));
        if (moveAngle < 0) moveAngle += 360;

        // 根据距离调整功率
        double power;

        // 如果距离较远，使用全速
        if (currentDistance > slowDownDistance) {
            power = movePower;
        }
        // 如果距离中等，使用中等速度
        else if (currentDistance > distanceTolerance * 2) {
            power = movePower * 0.7;
        }
        // 如果距离很近，使用慢速
        else {
            power = movePower * 0.4;
        }

        // 最小功率限制，确保能移动
        if (currentDistance > 50) {
            power = Math.max(power, 0.3);
        }

        // 移动，不旋转
        drive.drive(-moveAngle, power, true, 0);
    }

    // 主要更新方法
    public boolean update() {
        // 如果已经完成，返回true
        if (currentState == State.COMPLETE) {
            return true;
        }

        // 更新IMU数据
        drive.getIMU().update();

        // 根据当前状态执行相应控制
        switch (currentState) {
            case ROTATING:
                rotateToHeading();
                break;
            case MOVING:
                moveToPosition();
                break;
        }

        // 检查是否完成
        return isAtTarget();
    }

    // 立即停止机器人
    public void stop() {
        drive.drive(0, 0, false, 0);
        currentState = State.COMPLETE;
    }

    // 获取调试信息
    public double getCurrentHeadingError() {
        return currentHeadingError;
    }

    public double getCurrentDistance() {
        return currentDistance;
    }

    // 设置减速距离
    public void setSlowDownDistance(double distance) {
        this.slowDownDistance = distance;
    }

    // 获取当前目标信息
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getTargetHeading() { return targetHeading; }
}