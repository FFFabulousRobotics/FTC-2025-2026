package org.firstinspires.ftc.teamcode.test;

import static java.lang.Math.abs;
import static java.lang.Math.hypot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MotorPIDController;

@Autonomous(name = "Blue Auto Test")
public class BlueAutoTest extends OpMode {
    DcMotor leftFront, rightFront, leftRear, rightRear;
    GoBildaPinpointDriver pp;
    MecanumDrive drive;
    ZwhPathing pathing;
    ElapsedTime time = new ElapsedTime();
    DcMotor motor;
    MotorPIDController pid;
    DcMotor intake;
    Servo s1, s2, s3;

    boolean ONLAUNCH = false, ONPRESS = false;
    double launchtime, presstime, intaketime;
    double CURRENTX, CURRENTY, CURRENTTIME, RACESTARTTIME;
    public enum PathState {
        Pred,
        START,
        GOINTAKE1,
        DOINTAKE1,
        SHOOT1,
        GOINTAKE2,
        DOINTAKE2,
        SHOOT2,
        GOINTAKE3,
        DOINTAKE3,
        SHOOT3,
        END,
        Fied
    }
    PathState pathState;

    public void pathupdate() {
        switch (pathState) {
            case Pred:
                pathing.setTarget(-237.1,12.2,-5.26);
                pathState = PathState.START;
                break;
            case START:
                if(pathing.update()) {
                    shoot();
                    pathing.setTarget(-474.3,134.5,37.31);
                    pathState = PathState.GOINTAKE1;
                    pathing.setMaxPower(0.5);
                }
                break;
            case GOINTAKE1:
                if(pathing.update()) {
                    pathing.setTarget(-241.1,298.4,35.82);
                    pathState = PathState.DOINTAKE1;
                    intake.setPower(1.0);
                    intaketime = time.seconds();
                    pathing.setMaxPower(1.0);
                }
                break;
            case DOINTAKE1:
                if(pathing.update()) {
                    if(time.seconds() - intaketime >= 2) {
                        pathing.setTarget(-237.1,12.2,-5.26);
                        pathState = PathState.SHOOT1;
                        intake.setPower(0.0);
                    }
                }
                break;
            case SHOOT1:
                if(pathing.update()) {
                    if(!ONPRESS) {
                        shoot();
                        pathing.setTarget(-650.4, 300.1, 35.47);
                        pathState = PathState.GOINTAKE2;
                    }
                }
                break;
            case GOINTAKE2:
                if(pathing.update()) {
                    pathing.setTarget(-371.7,494.0,33.18);
                    pathState = PathState.DOINTAKE2;
                    intake.setPower(1.0);
                    intaketime = time.seconds();
                    pathing.setMaxPower(0.5);
                }
                break;
            case DOINTAKE2:
                if(pathing.update()) {
                    if(time.seconds() - intaketime >= 2) {
                        pathing.setTarget(-237.1,12.2,-5.26);
                        pathState = PathState.SHOOT2;
                        intake.setPower(0.0);
                        pathing.setMaxPower(1.0);
                    }
                }
                break;
            case SHOOT2:
                if(pathing.update()) {
                    shoot();
                    pathing.setTarget(-786.4,471.6,37.63);
                    pathState = PathState.GOINTAKE3;
                }
                break;
            case GOINTAKE3:
                if(pathing.update()) {
                    pathing.setTarget(-506.9,669.8,35.57);
                    pathState = PathState.DOINTAKE3;
                    intake.setPower(1.0);
                    intaketime = time.seconds();
                    pathing.setMaxPower(0.5);
                }
                break;
            case DOINTAKE3:
                if(pathing.update()) {
                    if(time.seconds() - intaketime >= 2) {
                        pathing.setTarget(-237.1,12.2,-5.26);
                        pathState = PathState.SHOOT3;
                        intake.setPower(0.0);
                        pathing.setMaxPower(1.0);
                    }
                }
                break;
            case SHOOT3:
                if(pathing.update()) {
                    shoot();
                    pathing.setTarget(-223.6,35.0,-0.25);
                    pathState = PathState.END;
                }
                break;
            case END:
                if(pathing.update()) {
                    telemetry.addLine("AUTONOMOUS FINISHED");
                    pathState = PathState.Fied;
                }
                break;
            case Fied:
                drive.drive(0,0,true,0);
                break;
        }
    }

    public void shoot() {
        drive.stop();
//      double preshoottime = time.seconds();
//      while(time.seconds() - preshoottime <= ) ;
        s1.setPosition(0.5);
        s2.setPosition(0.5);
        s3.setPosition(0.5);
        ONLAUNCH = true;
        launchtime = time.seconds();
    }

    public void overlaunch() {
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        ONLAUNCH = false;
        ONPRESS = true;
        presstime = time.seconds();
    }

    public void pressupdate() {
        if(time.seconds() - presstime <= 2) motor.setPower(-1.0);
        else {
            pid.setTarget(-1200);
            if(abs(pid.update()) <= 10) ONPRESS = false;
        }
    }

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "BR");
        pp = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // 设置功率因数（可以根据实际情况调整）
        double[] powerFactors = {1.0, 1.0, -1.0, -1.0};
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear, pp, powerFactors);
        double[] distancePIDParams = {5e-2, 4e-5, 0.01};
        double[] anglePIDParams = {8e-2, 4e-5, 0.01};
        pathing = new ZwhPathing(drive, 0,
                distancePIDParams, anglePIDParams,
                1,  // 最大功率
                4.0,  // 旋转容差（度）
                25.0  // 距离容差（mm）
        );
        // 初始化电机
        motor = (DcMotor) (hardwareMap.get("PRESS"));
        pid = new MotorPIDController(motor, 0.01, 0.0000001, 0.0005);
        intake = (DcMotor) hardwareMap.get("IN");
        s1 = (Servo) (hardwareMap.get("servo0"));
        s2 = (Servo) (hardwareMap.get("servo1"));
        s3 = (Servo) (hardwareMap.get("servo2"));
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);

        pathState = PathState.Pred;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0.8);
        double CurrentTime = time.seconds();
        while(time.seconds() - CurrentTime < 0.5) motor.setPower(0.8- (time.seconds() - CurrentTime) * 1.6);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid.setTarget(-1200);
        ElapsedTime pidtimer = new ElapsedTime();
        while(abs(motor.getCurrentPosition() + 1200) > 5 && pidtimer.seconds() < 1.0) pid.update();
        motor.setPower(0.0);

        drive.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        CURRENTX = 0;
        CURRENTY = 0;
        CURRENTTIME = time.seconds();
        RACESTARTTIME = time.seconds();
        ONLAUNCH = false;
        ONPRESS = false;
        pathState = PathState.Pred;
    }

    @Override
    public void loop() {
        double CURRENTSPEED, dx, dt;
        dx = hypot(drive.getIMU().getPosX(DistanceUnit.MM) - CURRENTX, drive.getIMU().getPosY(DistanceUnit.MM) - CURRENTY);
        dt = time.seconds() - CURRENTTIME;
        if(dt <= 0.001) dt = 0.001;
        CURRENTSPEED = dx / dt;
        CURRENTX = drive.getIMU().getPosX(DistanceUnit.MM);
        CURRENTY = drive.getIMU().getPosY(DistanceUnit.MM);
        CURRENTTIME = time.seconds();

        if(!ONLAUNCH) pathupdate();

        if(time.seconds() - launchtime >= 0.5 && ONLAUNCH) overlaunch();
        if(time.seconds() - RACESTARTTIME >= 28.5 || !ONPRESS) {
            pid.setTarget(-1200);
            pid.update();
        }
        else if(ONPRESS) pressupdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("speed", CURRENTSPEED);
        telemetry.addData("x", drive.getIMU().getPosX(DistanceUnit.MM));
        telemetry.addData("y", drive.getIMU().getPosY(DistanceUnit.MM));
        telemetry.addData("heading", drive.getIMU().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Press Current Position", motor.getCurrentPosition());
        telemetry.update();
    }
}
