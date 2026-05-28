package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.test.ZwhPathing;

@Autonomous
public class BlueAuto extends LinearOpMode{
    public void shoot(Servo s1,Servo s2,Servo s3)
    {
        s1.setPosition(0.5);
        sleep(100);
        s3.setPosition(0.5);
        sleep(100);
        s2.setPosition(0.5);
        sleep(100);
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
    }
    public void safe(ZwhPathing pathing) {
        pathing.setTarget(-546.7,3.3,37.50);
        while(!pathing.update())
            sleep(10);
    }
    public void reset_press(MotorPIDController pid) {
        ElapsedTime time = new ElapsedTime();
        double t0 = time.seconds();
        //rotate in one direction indefinitely
        pid.setTarget(10000);
        while(time.seconds()-t0<0.5)
            pid.update();
        //reset encoder
        pid.controlled_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pid.controlled_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void step(boolean open,double x0,double y0,double x1,double y1,ZwhPathing pathing,DcMotor intake,Servo s1,Servo s2,Servo s3,MotorPIDController pid) {
        ElapsedTime time = new ElapsedTime();
        double t0 = time.seconds(), t1 = time.seconds();
        //press shooter
        pid.setTarget(-7000);
        safe(pathing);
        pathing.setTarget(x0,y0,38.26);
        while(!pathing.update())
        {
            if(time.seconds()-t0<2.0)
                pid.update();
            sleep(10);
            t1 = time.seconds();
        }
        while(time.seconds()-t1<2.0)
            pid.update();
        intake.setPower(1);
        pid.setTarget(-1200);
        while(abs(pid.update())>=200)
            sleep(5);
        pathing.setMaxPower(0.6);
        //go forward and get the artifacts
        pathing.setTarget(x1,y1,38.26);
        while(!pathing.update())
            sleep(10);
        pathing.setMaxPower(1.0);
        sleep(1000);
        //stop intake
        intake.setPower(0);
        /*if(open) {
            pathing.setTarget(-390.3,-336.5,51.33);
            sleep(100);
            pathing.setTarget(-294.5, -400.8, 51.33);
            while(!pathing.update())
                sleep(10);
        }*/
        sleep(200);
        pathing.setTarget(-212.2,41.9,-10.55);
        while(!pathing.update())
            sleep(10);
        shoot(s1,s2,s3);
    }
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "BR");
        GoBildaPinpointDriver pp = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // 设置功率因数（可以根据实际情况调整）
        double[] powerFactors = {1.0, 1.0, -1.0, -1.0}; // 左前, 右前, 左后, 右后
        MecanumDrive drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear, pp, powerFactors);
        ;
        ZwhPathing pathing;
        ElapsedTime runtime = new ElapsedTime();
        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double[] distancePIDParams = {5e-2, 4e-5, 0.01};
        double[] anglePIDParams = {8e-2, 4e-5, 0.01};
        pathing = new ZwhPathing(drive, 0,
                distancePIDParams, anglePIDParams,
                1,  // 最大功率
                2.0,  // 旋转容差（度）
                10.0  // 距离容差（mm）
        );
        // 初始化电机
        DcMotor motor = (DcMotor) (hardwareMap.get("PRESS"));
        MotorPIDController pid = new MotorPIDController(motor, 0.1, 0.000005, 0.0005);
        reset_press(pid);
        DcMotor intake = (DcMotor) hardwareMap.get("IN");
        Servo s1 = (Servo) (hardwareMap.get("servo0"));
        Servo s2 = (Servo) (hardwareMap.get("servo1"));
        Servo s3 = (Servo) (hardwareMap.get("servo2"));
        Servo brush = (Servo) (hardwareMap.get("Brush"));
        brush.setPosition(1);
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        ElapsedTime time = new ElapsedTime();
        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        //go to shooting pos and shoot
        pathing.setTarget(-212.2,41.9,-10.55);
        while (!pathing.update())
            sleep(10);
        shoot(s1, s2, s3);
        sleep(50);
        step(true,-425.9,150.9,-249.3,285.0,pathing,intake,s1,s2,s3,pid);
        sleep(100);
        step(false,-596.5,310.0,-382.1,474.5,pathing,intake,s1,s2,s3,pid);
        sleep(100);
        step(false,-693.8,523.0,-516.1,660.8,pathing,intake,s1,s2,s3,pid);
        pathing.setTarget(-223.6,35.0,-0.25);
    }
}
