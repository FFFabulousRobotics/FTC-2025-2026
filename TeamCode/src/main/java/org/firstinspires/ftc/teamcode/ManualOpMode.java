
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Mecanum Drive Calibrated")
public class ManualOpMode extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        // 初始化电机
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "BR");

        // 功率因数校准数组 [左前, 右前, 左后, 右后]
        // 这些值需要根据实际测试进行调整
        // 例如，如果左前轮比其它轮子慢，可以将其因数设为1.1
        double[] powerFactors = {1.0, 1.0, -1.0, -1.0};

        // 创建驱动对象
        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear, powerFactors);

        // 设置电机模式
        drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // 从游戏手柄获取输入
            double gamepadX = gamepad1.left_stick_x;
            double gamepadY = -gamepad1.left_stick_y;  // 反转Y轴
            double rotation = gamepad1.right_stick_x;

            // 计算相对于机器人的方位角和功率
            double heading = Math.toDegrees(Math.atan2(gamepadX, gamepadY));
            double power = Math.sqrt(gamepadX * gamepadX + gamepadY * gamepadY);

            // 如果摇杆在中心附近，停止运动
            if (power < 0.1) {
                power = 0;
            }

            // 驱动机器人
            drive.drive(heading, power, rotation);

            // 调试信息
            telemetry.addData("Heading", "%.1f°", heading);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Rotation", "%.2f", rotation);
            telemetry.addData("Power Factors",
                    "LF:%.2f, RF:%.2f, LR:%.2f, RR:%.2f",
                    drive.getPowerFactor(0), drive.getPowerFactor(1),
                    drive.getPowerFactor(2), drive.getPowerFactor(3));
            telemetry.update();
        }

        drive.stop();
    }
}