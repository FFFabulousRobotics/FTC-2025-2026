package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class ShbbyAutoForBlue extends LinearOpMode{

    private MecanumDrive drive;
    public void runOpMode() {
        // 初始化电机
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "BR");
        DcMotor motor = (DcMotor) (hardwareMap.get("PRESS"));
        MotorPIDController pid = new MotorPIDController(motor, 0.1, 0.000005, 0.0005);
        DcMotor intake = (DcMotor) hardwareMap.get("IN");
        DcMotor left = hardwareMap.get(DcMotor.class, "LL");
        DcMotor right = hardwareMap.get(DcMotor.class, "LR");

        float lift = 0;
        MotorPIDController left_pid = new MotorPIDController(left, 0.1, 0.00005, 0.0005);
        MotorPIDController right_pid = new MotorPIDController(right, 0.1, 0.00005, 0.0005);
        Servo s1 = (Servo) (hardwareMap.get("servo0"));
        Servo s2 = (Servo) (hardwareMap.get("servo1"));
        Servo s3 = (Servo) (hardwareMap.get("servo2"));
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        ElapsedTime time = new ElapsedTime();
        double servoTime = 0, motorTime = 0;
        // 功率因数校准数组 [左前, 右前, 左后, 右后]
        // 这些值需要根据实际测试进行调整
        // 例如，如果左前轮比其它轮子慢，可以将其因数设为1.1
        double[] powerFactors = {1.0, 1.0, -1.0, -1.0};

        // 创建驱动对象
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear, (GoBildaPinpointDriver) hardwareMap.get("odo"), powerFactors);

        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        drive.drive(180, 1, false, 0);
        sleep(400);
        drive.stop();
        sleep(500);
        s1.setPosition(0.5);
        s2.setPosition(0.5);
        s3.setPosition(0.5);
        servoTime = time.seconds();
        //if (time.seconds() - servoTime >= 0.1) {
        sleep(1000);
        s1.setPosition(0.4);
        s2.setPosition(0.6);
        s3.setPosition(0.6);
        drive.drive(-90, 1, false, 0);
        sleep(600);
    }
}