package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MotorPIDController;
import org.firstinspires.ftc.teamcode.PedroPathing_forBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class PedroPathingTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, OpModetimer;

    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private DcMotor motor, intake;
    private Servo s1, s2, s3;

    private GoBildaPinpointDriver pinpoint;
    private MotorPIDController pid;
    private double servoTime = 0, motorTime = -1.5;

    // 功率因数校准数组 [左前, 右前, 左后, 右后]
    private double[] powerFactors = {1.0, 1.0, -1.0, -1.0};

    private boolean islaunching = false, launchDONE = false, pressDONE = false;
    private double launchstarttime = 0;
    ElapsedTime time = new ElapsedTime();

    private boolean launchcheck() {
        if(islaunching && time.seconds() - launchstarttime > 0.1) {
            s1.setPosition(0.4);
            s2.setPosition(0.6);
            s3.setPosition(0.6);
            islaunching = false;
            launchDONE = true;
            pressDONE = false;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }
        return false;
    }

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "BR");
        motor = hardwareMap.get(DcMotor.class, "PRESS");
        intake = hardwareMap.get(DcMotor.class, "IN");
        s1 = hardwareMap.get(Servo.class, "servo0");
        s2 = hardwareMap.get(Servo.class, "servo1");
        s3 = hardwareMap.get(Servo.class, "servo2");
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pid = new MotorPIDController(motor, 0.01, 0.0000001, 0.0005);


        time = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void loop() {
        Pose currentPose = follower.getPose();

        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        double currentHeading = currentPose.getHeading();

        telemetry.addData("X", currentX);
        telemetry.addData("Y", currentY);
        telemetry.addData("Heading", Math.toDegrees(currentHeading));
    }
}
