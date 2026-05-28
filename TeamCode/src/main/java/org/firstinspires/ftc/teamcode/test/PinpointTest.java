package org.firstinspires.ftc.teamcode.test;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.*;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
//
@TeleOp(name = "GoBilda Pinpoint Test", group = "测试")
public class PinpointTest extends LinearOpMode {

    private GoBildaPinpointDriver pinpointDriver;
    @Override
    public void runOpMode() {
        // 1. 初始化设备
        // 假设在配置中给设备命名为 "odo"，并连接到I2C端口3
        pinpointDriver = (GoBildaPinpointDriver) hardwareMap.get("odo");;

        telemetry.addData("状态", "硬件已初始化");
        telemetry.update();

        // 2. 配置设备参数 (根据你的机器人机械结构修改)
        // 设置吊舱偏移量（单位：毫米）。这是吊舱相对于你希望追踪的“机器人中心”的位置。
        // 例如：X吊舱（负责前进方向）在中心左侧100mm，Y吊舱（负责平移方向）在中心前方150mm
        pinpointDriver.setOffsets(-30, 15, DistanceUnit.MM);

        // 设置编码器分辨率（如果你使用的是goBilda的摆臂式吊舱）
//        pinpointDriver.setEncoderResolution(GoBildaOdometryPods.goBILDA_SWINGARM_POD);
//        GoBildaOdometryPods.goBILDA_SWINGARM_POD
        pinpointDriver.setEncoderResolution(52, DistanceUnit.MM);

        // 设置编码器方向（根据安装情况调整，如果数值反向则修改）
        pinpointDriver.setEncoderDirections(EncoderDirection.REVERSED, EncoderDirection.REVERSED);

        // 3. 等待开始，并执行初始校准
        telemetry.addData("状态", "准备校准，请确保机器人静止");
        telemetry.addData("提示", "按START开始校准并启动");
        telemetry.update();

        waitForStart();

        // 启动时重置位置为(0,0,0)并校准IMU（机器人必须静止！）
        telemetry.addData("状态", "正在校准IMU...");
        telemetry.update();
        pinpointDriver.resetPosAndIMU(); // 此过程约0.25秒
        sleep(300); // 稍作等待确保校准完成
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "FL");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "FR");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "BL");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "BR");
        double xPosMMmax = -200, xPosMMmin = 200, yPosMMmax = -200, yPosMMmin = 200;
        MecanumDrive drive=new MecanumDrive(leftFront,rightFront,leftRear,rightRear, (GoBildaPinpointDriver) hardwareMap.get("odo"), new double[]{1.0, 1.0, -1.0, -1.0});
        // 4. 主循环 - 读取并显示数据
        while (opModeIsActive()) {

            double gamepadX = -gamepad1.left_stick_x;
            double gamepadY = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;
            telemetry.addData("angle:",Math.toDegrees(Math.atan2(gamepadX,gamepadY)));
            // 计算相对于机器人的方位角和功率
            double heading = Math.toDegrees(Math.atan2(gamepadX, gamepadY));
            double power = Math.sqrt(gamepadX * gamepadX + gamepadY * gamepadY);

            // 如果摇杆在中心附近，停止运动
            if (power < 0.1) {
                power = 0;
            }

            // 驱动机器人
            drive.drive(heading, power, true,rotation);
            // 4.1 必须调用update()来获取新数据！
            pinpointDriver.update();

            // 4.2 获取设备状态
            DeviceStatus status = pinpointDriver.getDeviceStatus();
            String statusMessage = getStatusString(status);

            // 4.3 获取位置和航向（多种方式）
            // 方式一：获取Pose2D对象（包含X,Y和Heading）
            Pose2D currentPose = pinpointDriver.getPosition();
            double xPosMM = currentPose.getX(DistanceUnit.MM);
            double yPosMM = currentPose.getY(DistanceUnit.MM);
            double headingRad = currentPose.getHeading(AngleUnit.RADIANS);
            double headingDeg = currentPose.getHeading(AngleUnit.DEGREES);

            // 4.4 获取其他信息
            double loopFreq = pinpointDriver.getFrequency();
            int loopTime = pinpointDriver.getLoopTime();

            // 4.5 在Driver Hub上显示所有信息
            telemetry.addLine("=== 设备状态 ===");
            telemetry.addData("状态", statusMessage);
            telemetry.addData("循环频率", "%.0f Hz", loopFreq);
            telemetry.addData("循环时间", "%d μs", loopTime);

            telemetry.addLine("=== 位置与航向 ===");
            telemetry.addData("X 位置", "%.1f mm", xPosMM);
            telemetry.addData("Y 位置", "%.1f mm", yPosMM);
            telemetry.addData("航向角", "%.2f° (%.2f rad)", headingDeg, headingRad);
            xPosMMmax = max(xPosMMmax, xPosMM);
            xPosMMmin = min(xPosMMmin, xPosMM);
            yPosMMmax = max(yPosMMmax, yPosMM);
            yPosMMmin = min(yPosMMmin, yPosMM);
            telemetry.addData("X 位置max", "%.1f mm", xPosMMmax);
            telemetry.addData("X 位置min", "%.1f mm", xPosMMmin);
            telemetry.addData("Y 位置max", "%.1f mm", yPosMMmax);
            telemetry.addData("Y 位置min", "%.1f mm", yPosMMmin);

            // 4.6 根据设备状态给出警告
            if (status != DeviceStatus.READY) {
                telemetry.addLine("\n⚠️ 警告：设备未就绪！");
                telemetry.addData("可能原因", "吊舱未连接或校准失败");
                telemetry.addData("LED颜色参考", "红色:未就绪/校准中, 紫色:无吊舱");
                telemetry.addData("蓝色: X吊舱故障", "橙色: Y吊舱故障");
            }

            telemetry.update();

            // 控制循环频率，避免Telemetry刷新过快
            sleep(50); // 约20Hz更新率
        }

        // 5. 停止时显示最终位置
        telemetry.addData("程序结束", "最终位置: (%.1f, %.1f) mm", pinpointDriver.getPosX(DistanceUnit.MM), pinpointDriver.getPosY(DistanceUnit.MM));
        telemetry.update();
        sleep(2000);
    }

    /**
     * 将设备状态枚举转换为可读的字符串
     */
    private String getStatusString(DeviceStatus status) {
        switch (status) {
            case NOT_READY:
                return "未就绪 (正在启动)";
            case READY:
                return "就绪 (正常运行)";
            case CALIBRATING:
                return "校准中";
            case FAULT_NO_PODS_DETECTED:
                return "故障：未检测到吊舱";
            case FAULT_X_POD_NOT_DETECTED:
                return "故障：未检测到X吊舱";
            case FAULT_Y_POD_NOT_DETECTED:
                return "故障：未检测到Y吊舱";
            default:
                return "未知状态";
        }
    }
}