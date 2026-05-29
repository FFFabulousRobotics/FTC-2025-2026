package org.firstinspires.ftc.teamcode.test;

import static java.lang.Math.abs;

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

@Autonomous
public class ShabbyTest extends OpMode {
    DcMotor leftFront, rightFront, leftRear, rightRear;
    GoBildaPinpointDriver pp;
    double[] powerFactors;
    MecanumDrive drive;
    ZwhPathing pathing;
    ElapsedTime time = new ElapsedTime();
    double[] distancePIDParams, anglePIDParams;
    // 初始化电机
    DcMotor motor;
    MotorPIDController pid;
    DcMotor intake;
    Servo s1, s2, s3;

    boolean ONLAUNCH = false, ONPRESS = false;
    double launchtime, presstime;
    public enum PathState {
        Pred,
        START,
        END
    }
    PathState pathState;

    public void pathupdate() {
        switch (pathState) {
            case Pred:
                pathing.setTarget(-212.2,41.9,-10.55);
                pathState = PathState.START;
                break;
            case START:
                if(pathing.update()) {
                    shoot();
                    pathState = PathState.END;
                }
                break;
            case END:
                telemetry.addLine("AUTONOMOUS FINISHED");
                break;
        }
    }

    public void shoot() {
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
        if(time.seconds() - presstime <= 1.5) motor.setPower(-1.0);
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
        while(time.seconds() - CurrentTime < 0.5) ;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pid.setTarget(-1200);
        ElapsedTime pidtimer = new ElapsedTime();
        while(abs(pid.update()) > 5 && pidtimer.seconds() < 0.5) ;
        motor.setPower(0.0);

        drive.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        pathupdate();

        if(time.seconds() - launchtime >= 0.1 && ONLAUNCH) overlaunch();
        if(ONPRESS) pressupdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", drive.getIMU().getPosX(DistanceUnit.MM));
        telemetry.addData("y", drive.getIMU().getPosY(DistanceUnit.MM));
        telemetry.addData("heading", drive.getIMU().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Press Current Position", motor.getCurrentPosition());
        telemetry.update();
    }
}
