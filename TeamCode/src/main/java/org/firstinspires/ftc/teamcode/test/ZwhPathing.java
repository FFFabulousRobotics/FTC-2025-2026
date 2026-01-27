package org.firstinspires.ftc.teamcode.test;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.GenericPIDController;

public class ZwhPathing {
    private final MecanumDrive md;
    private final double offset;

    // 位置PID控制器
    private final GenericPIDController distancePID;
    private final GenericPIDController anglePID;
    // 目标状态
    private double targetX, targetY;
    private double targetHeading;

    // 控制参数
    private final double maxPower;
    private final double rotationTolerance;  // 旋转容差(度)
    private final double distanceTolerance;  // 距离容差(mm)
    // 状态跟踪
    private double X, Y;
    private double Heading;
    private double startX;
    private double startY;
    double[] dParams, aParams;
    public double dx,dy,moveHeading,movePower,rotationPower;
    boolean state=false;//false:far, true:near, reduce PID coefficients
    // 自定义控制参数
    public ZwhPathing(MecanumDrive drive, double offsetDegrees,
                      double[] distancePIDParams, double[] anglePIDParams,
                      double maxPower, double rotationTolerance,
                      double distanceTolerance) {
        dParams=distancePIDParams;
        aParams=anglePIDParams;
        this.md = drive;
        drive.getIMU().resetPosAndIMU();
        sleep(1000);
        offset = offsetDegrees;

        // 初始化PID控制器
        distancePID = new GenericPIDController(
                distancePIDParams[0], distancePIDParams[1], distancePIDParams[2]);
        distancePID.setMaxPower(maxPower);

        anglePID = new GenericPIDController(
                anglePIDParams[0], anglePIDParams[1], anglePIDParams[2]);
        anglePID.setMaxPower(0.5);

        this.maxPower = maxPower;
        this.rotationTolerance = rotationTolerance;
        this.distanceTolerance = distanceTolerance;
        updateRobotState();
    }

    // 设置目标位置（不改变朝向）
    public void setTarget(double x, double y) {
        startX=md.getIMU().getPosX(DistanceUnit.MM);
        startY=md.getIMU().getPosY(DistanceUnit.MM);
        targetX = x;
        targetY = y;
        targetHeading = md.getIMU().getHeading(AngleUnit.DEGREES);
        resetPIDControllers();
        distancePID.setPidCoefficients(dParams[0],dParams[1],dParams[2]);
        distancePID.setTarget(0);
        anglePID.setTarget(0);
        state=false;
    }

    // 设置目标位置和朝向
    public void setTarget(double x, double y, double headingDegrees) {
        startX=md.getIMU().getPosX(DistanceUnit.MM);
        startY=md.getIMU().getPosY(DistanceUnit.MM);
        targetX = x;
        targetY = y;
        targetHeading = headingDegrees;
        resetPIDControllers();
        distancePID.setPidCoefficients(dParams[0],dParams[1],dParams[2]);
        distancePID.setTarget(0);
        anglePID.setTarget(0);
        state=false;
    }

    // 设置PID参数
    public void setDistancePIDParams(double kp, double ki, double kd) {
        distancePID.setPidCoefficients(kp, ki, kd);
        dParams=new double[]{kp,ki,kd};
    }

    public void setAnglePIDParams(double kp, double ki, double kd) {
        anglePID.setPidCoefficients(kp, ki, kd);
        aParams=new double[]{kp,ki,kd};
    }

    // 重置PID控制器
    private void resetPIDControllers() {
        distancePID.reset();
        anglePID.reset();
        updateRobotState();  // 更新初始状态
    }

    // 更新机器人状态（位置、速度等）
    private void updateRobotState() {
        md.getIMU().update();
        X=md.getIMU().getPosX(DistanceUnit.MM);
        Y=md.getIMU().getPosY(DistanceUnit.MM);
        Heading=md.getIMU().getHeading(AngleUnit.DEGREES)-offset;
    }
    public double getDistanceToTarget()
    {
        return hypot(X-targetX,Y-targetY);
    }
    public double getHeadingError()
    {
        return targetHeading-Heading;
    }
    // 角度规范化到-180到180度
    private double normalizeAngle180(double angle) {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    // 角度规范化到0-360度
    private double normalizeAngle360(double angle) {
        angle = angle % 360;
        if (angle < 0) angle += 360;
        return angle;
    }
    // 更新移动（返回是否到达目标）
    public boolean update() {
        updateRobotState();
        double dx = targetX-X;
        double dy = targetY-Y;
        this.dx=dx;
        this.dy=dy;
        double moveHeading=toDegrees(atan2(dy,dx));
        this.moveHeading=moveHeading;
        double distance=hypot(dx,dy);
        double angleError = normalizeAngle180(targetHeading-Heading);
        double rotationPower=anglePID.update(angleError);
        double movePower;
        // 检查是否到达目标
        if (abs(distance) < distanceTolerance &&abs(angleError)< rotationTolerance) {
            md.drive(0, 0, true, 0);
            return true;
        }
        if(!state){
            if(abs(distance)<distanceTolerance*2) {
                distancePID.reset();
                state=true;
                distancePID.setPidCoefficients(dParams[0]*0.5,dParams[1]*0.5,dParams[2]);
            }
            movePower=-distancePID.update(distance);
            movePower = Math.max(-maxPower, Math.min(maxPower, movePower));
        }
        else{
            movePower=-distancePID.update(distance);
            movePower = Math.max(-maxPower*0.8, Math.min(maxPower*0.8, movePower));
        }
        // 使用绝对角度模式（移动方向是场地绝对坐标系）
        md.drive(moveHeading, movePower, true, rotationPower);
        return false;
    }
    public boolean isAtTarget() {
        double currentX = md.getIMU().getPosX(DistanceUnit.MM);
        double currentY = md.getIMU().getPosY(DistanceUnit.MM);
        double distance = hypot(targetX - currentX, targetY - currentY);
        double heading = md.getIMU().getHeading(AngleUnit.DEGREES);
        double angleError = normalizeAngle180(targetHeading - heading);
        if (distance > distanceTolerance||angleError > rotationTolerance) return false;
        return true;
    }
}