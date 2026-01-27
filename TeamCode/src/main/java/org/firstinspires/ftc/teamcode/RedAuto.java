package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.random;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.test.ZwhPathing;
@Autonomous
public class RedAuto extends LinearOpMode{
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "BR");
        GoBildaPinpointDriver pp = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // 设置功率因数（可以根据实际情况调整）
        double[] powerFactors = {1.0, 1.0, -1.0, -1.0}; // 左前, 右前, 左后, 右后
        MecanumDrive drive=new MecanumDrive(leftFront, rightFront, leftRear, rightRear, pp, powerFactors);;
        ZwhPathing pathing;
        ElapsedTime runtime = new ElapsedTime();
        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // 或者使用自定义PID参数（取消注释以下代码使用自定义参数）
        double[] distancePIDParams = {2e-3, 2e-5, 0.01};
        double[] anglePIDParams = {5e-2, 1e-5, 0.005};
        pathing = new ZwhPathing(drive, 0,
                distancePIDParams, anglePIDParams,
                1,  // 最大功率
                2.0,  // 旋转容差（度）
                50.0  // 距离容差（mm）
        );
        // 初始化电机
        DcMotor motor = (DcMotor) (hardwareMap.get("PRESS"));
        MotorPIDController pid = new MotorPIDController(motor, 0.1, 0.000005, 0.0005);
        DcMotor intake = (DcMotor) hardwareMap.get("IN");
        Servo s1 = (Servo) (hardwareMap.get("servo0"));
        Servo s2 = (Servo) (hardwareMap.get("servo1"));
        Servo s3 = (Servo) (hardwareMap.get("servo2"));
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        ElapsedTime time = new ElapsedTime();
        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        pathing.setTarget(-235,-11.7,0);
        while(!pathing.update())
            sleep(10);
        s1.setPosition(0.5);
        s3.setPosition(0.5);
        sleep(100);
        s2.setPosition(0.5);
        sleep(100);
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        sleep(50);
        pid.setTarget(-4500);
        while(abs(pid.update())>=500)
            pid.setTarget(-500);
        pathing.setTarget(-443,-167.4,-32.51);
        while(!pathing.update())
            sleep(10);
        intake.setPower(1);
        sleep(2);
        pathing.setTarget(-239.5,-293.7,-38.6);
        while(!pathing.update())
            sleep(10);
        intake.setPower(0);
            sleep(2);
        pathing.setTarget(-313.7,-460.5,55.24);
        while(!pathing.update())
            sleep(10);
        pathing.setTarget(-235,-11.7,0);
        while(!pathing.update())
            sleep(10);
        s1.setPosition(0.5);
        s3.setPosition(0.5);
        sleep(100);
        s2.setPosition(0.5);
        sleep(100);
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        sleep(1145);
        pid.setTarget(-4500);
        while(abs(pid.update())>=50)
            pid.setTarget(-500);
        pathing.setTarget(-580.4,-338.8,-35.69);
        while(!pathing.update())
            sleep(10);
        intake.setPower(1);
        pathing.setTarget(-303.9,-338.8,-39.69);
        while(!pathing.update())
            sleep(10);
        intake.setPower(0);
//        pathing.setTarget(333.2,533.2,39.69);
//        while(pathing.update());

        pathing.setTarget(-235,-11.7,0);
        while(!pathing.update())
            sleep(10);
        s1.setPosition(0.5);
        s3.setPosition(0.5);
        sleep(100);
        s2.setPosition(0.5);
        sleep(100);
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        sleep(1145);
        pid.setTarget(-4500);
        while(abs(pid.update())>=50)
            pid.setTarget(-500);
        pathing.setTarget(-704.8,-520.4,-33.69);
        while(!pathing.update())
            sleep(10);
        intake.setPower(1);
        pathing.setTarget(-441,-668.6,-33.69);
        while(!pathing.update())
            sleep(10);
        intake.setPower(0);
//        pathing.setTarget(467,711.4,-39.69);
//        while(pathing.update());
        pathing.setTarget(-235,-11.7,0);
        while(!pathing.update())
            sleep(10);
        s1.setPosition(0.5);
        s3.setPosition(0.5);
        sleep(100);
        s2.setPosition(0.5);
        sleep(100);
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        sleep(1145);
        pid.setTarget(-4500);
        while(abs(pid.update())>=50)
            pid.setTarget(-500);

    }
}
