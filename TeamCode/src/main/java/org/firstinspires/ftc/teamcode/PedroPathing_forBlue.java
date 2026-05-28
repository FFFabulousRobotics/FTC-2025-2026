package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class PedroPathing_forBlue extends OpMode {
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

    public enum PathState {
        START_LAUNCH,
        LAUNCH_pred0,
        LAUNCH_wait0,
        TO_INTAKE1,
        INTAKE1_fied,
        LAUNCH_pred1,
        LAUNCH_wait1,
        TO_INTAKE2,
        INTAKE2_fied,
        LAUNCH_pred2,
        LAUNCH_wait2,
        TO_INTAKE3,
        INTAKE3_fied,
        LAUNCH_pred3,
        LAUNCH_wait3,
        OPGATE,
        GATE_fied,
        ALL_fied
    }

    PathState pathstate;

    private final Pose startPose = new Pose(22,120,Math.toRadians(136));
    private final Pose launchPose = new Pose(34.58344640434193,108.04884667571235,Math.toRadians(136));
    private final Pose intake1_start = new Pose(38.49118046132972,84.40705563093623,Math.toRadians(180));
    private final Pose intake1_end = new Pose(15.230936227951153,84.40705563093623,Math.toRadians(180));
    private final Pose intake2_start = new Pose(38.29579375848033,60.37449118046133,Math.toRadians(180));
    private final Pose intake2_end = new Pose(15.230936227951153,60.37449118046133,Math.toRadians(180));
    private final Pose intake3_start = new Pose(38.10040705563094,36.146540027137036,Math.toRadians(180));
    private final Pose intake3_end = new Pose(15.230936227951153,36.146540027137036,Math.toRadians(180));
    private final Pose OpGate = new Pose(15.044776119402984,70.1438263229308,Math.toRadians(90));
    private final Pose Gate_end = new Pose(12.378561736770692,70.1438263229308,Math.toRadians(90));
    private final Pose All_end = new Pose(21.883310719131615,92.80868385345998,Math.toRadians(136));

    private PathChain startTOlaunch, TOintake1, ONintake1, TOlaunch1, TOintake2, ONintake2, TOgate, ONgate, Tolaunch2, TOintake3, ONintake3, TOlaunch3, TOfinish;
    public void BuildPaths() {
        startTOlaunch = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        TOintake1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intake1_start))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake1_start.getHeading())
                .build();
        ONintake1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1_start, intake1_end))
                .setLinearHeadingInterpolation(intake1_start.getHeading(), intake1_end.getHeading())
                .build();
        TOlaunch1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1_end, launchPose))
                .setLinearHeadingInterpolation(intake1_end.getHeading(), launchPose.getHeading())
                .build();
        TOintake2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intake2_start))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake2_start.getHeading())
                .build();
        ONintake2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2_start, intake2_end))
                .setLinearHeadingInterpolation(intake2_start.getHeading(), intake2_end.getHeading())
                .build();
        TOgate = follower.pathBuilder()
                .addPath(new BezierLine(intake2_end, OpGate))
                .setLinearHeadingInterpolation(intake2_end.getHeading(), OpGate.getHeading())
                .build();
        ONgate = follower.pathBuilder()
                .addPath(new BezierLine(OpGate,Gate_end))
                .setLinearHeadingInterpolation(OpGate.getHeading(), Gate_end.getHeading())
                .build();
        Tolaunch2 = follower.pathBuilder()
                .addPath(new BezierLine(Gate_end, launchPose))
                .setLinearHeadingInterpolation(Gate_end.getHeading(), launchPose.getHeading())
                .build();
        TOintake3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, intake3_start))
                .setLinearHeadingInterpolation(launchPose.getHeading(), intake3_start.getHeading())
                .build();
        ONintake3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3_start, intake3_end))
                .setLinearHeadingInterpolation(intake3_start.getHeading(), intake3_end.getHeading())
                .build();
        TOlaunch3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3_end, launchPose))
                .setLinearHeadingInterpolation(intake3_end.getHeading(), launchPose.getHeading())
                .build();
        TOfinish = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, All_end))
                .setLinearHeadingInterpolation(launchPose.getHeading(), All_end.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathstate) {
            case START_LAUNCH:
                follower.followPath(startTOlaunch,true);
                pathstate = PathState.LAUNCH_pred0;
                break;
            case LAUNCH_pred0:
                if(!follower.isBusy()) {
                    launchcontroller();
                    pathstate = PathState.LAUNCH_wait0;
                }
                break;
            case LAUNCH_wait0:
                if(launchcheck()) {
                    follower.followPath(TOintake1,true);
                    pathstate = PathState.TO_INTAKE1;
                }
                break;
            case TO_INTAKE1:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(ONintake1,true);
                    pathstate = PathState.INTAKE1_fied;
                }
                break;
            case INTAKE1_fied:
                if(!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(TOlaunch1,true);
                    pathstate = PathState.LAUNCH_pred1;
                }
                break;
            case LAUNCH_pred1:
                if(!follower.isBusy()) {
                    launchcontroller();
                    pathstate = PathState.LAUNCH_wait1;
                }
                break;
            case LAUNCH_wait1:
                if(launchcheck()) {
                    follower.followPath(TOintake2,true);
                    pathstate = PathState.TO_INTAKE2;
                }
                break;
            case TO_INTAKE2:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(ONintake2,true);
                    pathstate = PathState.INTAKE2_fied;
                }
                break;
            case INTAKE2_fied:
                if(!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(TOgate,true);
                    pathstate = PathState.OPGATE;
                }
                break;
            case OPGATE:
                if(!follower.isBusy()) {
                    follower.followPath(ONgate,true);
                    pathstate = PathState.GATE_fied;
                }
                break;
            case GATE_fied:
                if(!follower.isBusy()) {
                    follower.followPath(Tolaunch2,true);
                    pathstate = PathState.LAUNCH_pred2;
                }
                break;
            case LAUNCH_pred2:
                if(!follower.isBusy()) {
                    launchcontroller();
                    pathstate = PathState.LAUNCH_wait2;
                }
                break;
            case LAUNCH_wait2:
                if(launchcheck()) {
                    follower.followPath(TOintake3,true);
                    pathstate = PathState.TO_INTAKE3;
                }
                break;
            case TO_INTAKE3:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(ONintake3,true);
                    pathstate = PathState.INTAKE3_fied;
                }
                break;
            case INTAKE3_fied:
                if(!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(TOlaunch3,true);
                    pathstate = PathState.LAUNCH_pred3;
                }
                break;
            case LAUNCH_pred3:
                if(!follower.isBusy()) {
                    launchcontroller();
                    pathstate = PathState.LAUNCH_wait3;
                }
                break;
            case LAUNCH_wait3:
                if(launchcheck()) {
                    follower.followPath(TOfinish,true);
                    pathstate = PathState.ALL_fied;
                }
                break;
            case ALL_fied:
                if(!follower.isBusy()) {
                    telemetry.addLine("WELL DONE");
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
        }
    }

    public void setPathState(PathState newState) {
        pathstate = newState;
        pathTimer.resetTimer();
    }

    private void pressupdate() {
        if(motor.getCurrentPosition() > -4300 && !pressDONE) {
            motor.setPower(-1.0);
        }
        else {
            pressDONE = true;
            pid.setTarget(-1200);
            if(abs(motor.getCurrentPosition() + 1200) >= 10) {
                pid.update();
            }
            else {
                launchDONE = false;
            }
        }
    }
    private void launchcontroller() {
        s1.setPosition(0.5);
        s2.setPosition(0.5);
        s3.setPosition(0.5);
        islaunching = true;
        launchstarttime = time.seconds();
    }

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

        pathstate = PathState.START_LAUNCH;
        pathTimer = new Timer();
        OpModetimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        time = new ElapsedTime();

        BuildPaths();
        follower.setPose(startPose);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(1.0);
        double CurrentTime = time.seconds();
        while(time.seconds() - CurrentTime < 0.3) ;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid.setTarget(-1200);
        ElapsedTime pidtimer = new ElapsedTime();
        while(abs(pid.update()) > 5 && pidtimer.seconds() < 0.3) ;
        motor.setPower(0.0);
    }

    public void start() {
        OpModetimer.resetTimer();
        setPathState(pathstate);
    }

    @Override
    public void loop() {
        follower.update();
        if(launchDONE) pressupdate();
        statePathUpdate();

        telemetry.addData("path state",pathstate.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

        telemetry.addData("Press Current Position", motor.getCurrentPosition());
    }
}
