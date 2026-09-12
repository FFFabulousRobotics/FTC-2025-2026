package org.firstinspires.ftc.teamcode.test;
import android.graphics.Camera;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public interface RobotHardware {

    // basic movements
    void initMovement();

    void driveRobotFieldCentric(double axial, double lateral, double yaw);

    void driveRobot(double axial, double lateral, double yaw);

    RobotHardware setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower);

    void stopMotor();

    void setRunMode(DcMotor.RunMode mode);

    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior);

    void setTargetPosition(int moveCounts);

    void setStrafeTargetPosition(int moveCounts);

    boolean isAllBusy();

    RobotHardware setArmPower(double power);

    RobotHardware setLiftPower(double power);

    int getLiftPosition();

    RobotHardware setIntakePower(double power);

    RobotHardware setDumpPosition(double position);

    double getDumpPosition();

    RobotHardware setDronePosition(double position);

    RobotHardware setHolderPosition(double position);

    // computer vision
    void initDoubleVision();

    void closeVision();

    AprilTagDetection getAprilTag(int desiredTagId);

    void moveToAprilTag(AprilTagDetection desiredTag, double desiredDistance);

    void setTagDecimation(float decimation);

    //Recognition getTfod(String desiredLabel);

    //void moveToTfod(Recognition desiredTfod, double desiredWidthInScreen);

    // gyroscope
    double getHeading();

    double getHeading(AngleUnit unit);

    void resetYaw();

    // autonomous
    RobotHardware forward(double d);

    RobotHardware fastForward(double d, int step);

    RobotHardware fastForward(double d);

    RobotHardware backward(double d);

    RobotHardware fastBackward(double d, int step);

    RobotHardware fastBackward(double d);

    RobotHardware rightShift(double d);

    RobotHardware leftShift(double d);

    RobotHardware spin(double h);

    RobotHardware fastSpin(double h);

    RobotHardware sleep(long milliseconds);

    // utils
    RobotHardware driveStraight(double maxDriveSpeed, double distance, double heading);

    @Deprecated
    RobotHardware driveStrafe(double distance);

    RobotHardware driveStrafe(double maxDriveSpeed, double distance, double heading);

    void turnHead();

    RobotHardware turnToHeading(double maxTurnSpeed, double heading);

    void holdHeading(double maxTurnSpeed, double heading, double holdTime);

    double getSteeringCorrection(double desiredHeading, double proportionalGain);

    double getDistance();

    RobotHardware gotoDistance(double target_distance, double init_distance);

    double[] SpinVector(double[] vector, double angle);

    double[] getDisplacement(double[] CurrentPos, double[] DesiredPos);

    RobotHardware gotoPosition(double[] CurrentPos, double[] DesiredPos);

    RobotHardware gotoPosition(double x, double y, double h);

    RobotHardware gotoPosition2(double[] CurrentPos, double[] DesiredPos);

    RobotHardware gotoPosition2(double x, double y, double h);

    RobotHardware fastGotoPosition(double[] CurrentPos, double[] DesiredPos);

    RobotHardware fastGotoPosition(double x, double y, double h);

    void setDiagonalTargetPosition(int moveCounts, double angle);

    RobotHardware driveDiagonal(double maxDriveSpeed,
                                double distance,
                                int angle,
                                double heading);

    RobotHardware moveDiagonally(double distance, int angle);

    SparkFunOTOS.Pose2D getPosition();

    RobotHardware stretchArm();

    RobotHardware resetArm();

    RobotHardware moveDirect(double x, double y, double h);

    <T extends CameraControl> T getCameraControl(Class<T> controlType);

    VisionPortal.CameraState getCameraState();
}