package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@TeleOp(name = "ServoTest with HuskyLens", group = "Test")
public class ServoTest extends LinearOpMode {

    private Servo testServo1, testServo2, testServo3; // 三个舵机
    private I2cDeviceSynch huskyLens1; // 二哈识图1用于AprilTag识别
    private I2cDeviceSynch huskyLens2; // 二哈识图2用于颜色识别

    // 二哈识图1相关常量 (AprilTag识别)
    private static final I2cAddr HUSKYLENS1_ADDRESS = I2cAddr.create7bit(0x32);

    // 二哈识图2相关常量 (颜色识别)
    private static final I2cAddr HUSKYLENS2_ADDRESS = I2cAddr.create7bit(0x33);

    private static final int REGISTER_READ_START = 0x00;
    private static final int READ_LENGTH = 36;

    // AprilTag识别结果的数据偏移量
    private static final int OFFSET_FRAME_HEADER = 0;
    private static final int OFFSET_ALGORITHM_TYPE = 4;
    private static final int OFFSET_BLOCK_COUNT = 5;
    private static final int OFFSET_BLOCK_DATA_START = 6;
    private static final int BLOCK_DATA_LENGTH = 7;
    private static final int OFFSET_WITHIN_BLOCK_ID = 8;

    // 颜色区块信息
    private List<ColorBlock> colorBlocks;
    private String[] colorSequence;

    // 舵机角度常量 (358度舵机，30度对应的位置值)
    private static final double SERVO_30_DEGREES = 30.0 / 358.0;

    @Override
    public void runOpMode() {
        // 初始化硬件
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. 检测AprilTag并获取颜色序列
            detectAprilTagAndSetColors();

            // 2. 检测颜色区块并排序
            detectAndSortColorBlocks();

            // 3. 根据颜色序列控制舵机
            controlServosBasedOnColorSequence();

            // 显示当前状态
            displayTelemetry();

            telemetry.update();
            sleep(100); // 降低检测频率
        }
    }

    private void initializeHardware() {
        // 初始化三个舵机
        testServo1 = hardwareMap.get(Servo.class, "testServo1");
        testServo2 = hardwareMap.get(Servo.class, "testServo2");
        testServo3 = hardwareMap.get(Servo.class, "testServo3");

        // 初始化二哈识图1 (AprilTag识别)
        try {
            huskyLens1 = hardwareMap.get(I2cDeviceSynch.class, "huskylens1");
            huskyLens1.setI2cAddress(HUSKYLENS1_ADDRESS);
            huskyLens1.engage();
            telemetry.addData("HuskyLens1", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("HuskyLens1", "Initialization failed: " + e.getMessage());
        }

        // 初始化二哈识图2 (颜色识别)
        try {
            huskyLens2 = hardwareMap.get(I2cDeviceSynch.class, "huskylens2");
            huskyLens2.setI2cAddress(HUSKYLENS2_ADDRESS);
            huskyLens2.engage();
            telemetry.addData("HuskyLens2", "Initialized successfully");
        } catch (Exception e) {
            telemetry.addData("HuskyLens2", "Initialization failed: " + e.getMessage());
        }

        colorBlocks = new ArrayList<>();
    }

    private void detectAprilTagAndSetColors() {
        try {
            byte[] readBuffer = huskyLens1.read(REGISTER_READ_START, READ_LENGTH);

            if (readBuffer != null && isValidHeader(readBuffer)) {
                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;

                if (blockCount > 0) {
                    int blockDataStart = OFFSET_BLOCK_DATA_START;
                    int aprilTagId = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;

                    telemetry.addData("Detected AprilTag ID", aprilTagId);

                    setColorSequenceFromAprilTag(aprilTagId);
                } else {
                    colorSequence = null;
                    telemetry.addData("AprilTag Detection", "No tags found");
                }
            } else {
                colorSequence = null;
                telemetry.addData("AprilTag Detection", "Invalid data or no data");
            }
        } catch (Exception e) {
            colorSequence = null;
            telemetry.addData("AprilTag Detection Error", e.getMessage());
        }
    }

    private void detectAndSortColorBlocks() {
        colorBlocks.clear();

        try {
            byte[] readBuffer = huskyLens2.read(REGISTER_READ_START, READ_LENGTH);

            if (readBuffer != null && isValidHeader(readBuffer)) {
                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;

                for (int i = 0; i < blockCount && i < 3; i++) { // 最多处理3个区块
                    int blockDataStart = OFFSET_BLOCK_DATA_START + i * BLOCK_DATA_LENGTH;

                    if (blockDataStart + BLOCK_DATA_LENGTH <= readBuffer.length) {
                        int xCenter = ((readBuffer[blockDataStart] & 0xFF) << 8) | (readBuffer[blockDataStart + 1] & 0xFF);
                        int yCenter = ((readBuffer[blockDataStart + 2] & 0xFF) << 8) | (readBuffer[blockDataStart + 3] & 0xFF);
                        int width = ((readBuffer[blockDataStart + 4] & 0xFF) << 8) | (readBuffer[blockDataStart + 5] & 0xFF);
                        int height = ((readBuffer[blockDataStart + 6] & 0xFF) << 8) | (readBuffer[blockDataStart + 7] & 0xFF);
                        int id = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;

                        // 根据ID判断颜色 (需要根据你的颜色学习设置调整)
                        String color = (id == 1) ? "颜色1" : "颜色2";

                        colorBlocks.add(new ColorBlock(color, xCenter, yCenter, width, height, i + 1));
                    }
                }

                // 按X坐标排序 (从左到右)
                Collections.sort(colorBlocks, new Comparator<ColorBlock>() {
                    @Override
                    public int compare(ColorBlock b1, ColorBlock b2) {
                        return Integer.compare(b1.xCenter, b2.xCenter);
                    }
                });

                // 重新编号 (1, 2, 3)
                for (int i = 0; i < colorBlocks.size(); i++) {
                    colorBlocks.get(i).position = i + 1;
                }

            } else {
                telemetry.addData("Color Detection", "Invalid data or no data");
            }
        } catch (Exception e) {
            telemetry.addData("Color Detection Error", e.getMessage());
        }
    }

    private void controlServosBasedOnColorSequence() {
        if (colorSequence == null || colorBlocks.size() < 3) {
            telemetry.addData("Servo Control", "Waiting for AprilTag and 3 color blocks...");
            return;
        }

        // 重置所有舵机位置
        testServo1.setPosition(0);
        testServo2.setPosition(0);
        testServo3.setPosition(0);

        // 根据颜色序列控制对应的舵机
        for (int i = 0; i < 3; i++) {
            String expectedColor = colorSequence[i];

            // 找到对应位置的区块
            for (ColorBlock block : colorBlocks) {
                if (block.position == i + 1 && block.color.equals(expectedColor)) {
                    // 根据位置选择舵机并旋转30度
                    switch (i) {
                        case 0: // 最左边
                            testServo1.setPosition(SERVO_30_DEGREES);
                            telemetry.addData("Servo1", "Rotated 30° for " + expectedColor);
                            break;
                        case 1: // 中间
                            testServo2.setPosition(SERVO_30_DEGREES);
                            telemetry.addData("Servo2", "Rotated 30° for " + expectedColor);
                            break;
                        case 2: // 最右边
                            testServo3.setPosition(SERVO_30_DEGREES);
                            telemetry.addData("Servo3", "Rotated 30° for " + expectedColor);
                            break;
                    }
                    break;
                }
            }
        }
    }

    private void displayTelemetry() {
        if (colorSequence != null) {
            telemetry.addData("AprilTag Color Sequence",
                    colorSequence[0] + ", " + colorSequence[1] + ", " + colorSequence[2]);
        } else {
            telemetry.addData("AprilTag Color Sequence", "No AprilTag detected");
        }

        if (colorBlocks.size() > 0) {
            telemetry.addData("Detected Color Blocks", colorBlocks.size());
            for (ColorBlock block : colorBlocks) {
                telemetry.addData("Block " + block.position,
                        block.color + " at X:" + block.xCenter);
            }
        } else {
            telemetry.addData("Color Blocks", "No color blocks detected");
        }
    }

    private void setColorSequenceFromAprilTag(int aprilTagId) {
        switch (aprilTagId) {
            case 23:
                colorSequence = new String[]{"颜色1", "颜色2", "颜色2"};
                break;
            case 22:
                colorSequence = new String[]{"颜色2", "颜色1", "颜色2"};
                break;
            case 21:
                colorSequence = new String[]{"颜色2", "颜色2", "颜色1"};
                break;
            default:
                colorSequence = null;
                telemetry.addData("Unknown AprilTag ID", aprilTagId);
                break;
        }
    }

    private boolean isValidHeader(byte[] buffer) {
        return buffer != null &&
                buffer.length >= 2 &&
                buffer[OFFSET_FRAME_HEADER] == (byte) 0x55 &&
                buffer[OFFSET_FRAME_HEADER + 1] == (byte) 0xAA;
    }

    // 内部类用于存储颜色区块信息
    class ColorBlock {
        String color;
        int xCenter;
        int yCenter;
        int width;
        int height;
        int position; // 位置编号 (1, 2, 3)

        public ColorBlock(String color, int xCenter, int yCenter, int width, int height, int position) {
            this.color = color;
            this.xCenter = xCenter;
            this.yCenter = yCenter;
            this.width = width;
            this.height = height;
            this.position = position;
        }
    }
}
//package org.firstinspires.ftc.teamcode.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cAddr;
//
//@TeleOp(name = "ServoTest with HuskyLens", group = "Test")
//public class ServoTest extends LinearOpMode {
//
//    private Servo testServo;
//    private I2cDeviceSynch huskyLens;
//
//    // 二哈识图相关常量
//    private static final I2cAddr HUSKYLENS_ADDRESS = I2cAddr.create7bit(0x32);
//    private static final int REGISTER_READ_START = 0x00;
//    private static final int READ_LENGTH = 36;
//
//    // AprilTag识别结果的数据偏移量（需要根据二哈识图协议调整）
//    private static final int OFFSET_FRAME_HEADER = 0;
//    private static final int OFFSET_ALGORITHM_TYPE = 4;
//    private static final int OFFSET_BLOCK_COUNT = 5;
//    private static final int OFFSET_BLOCK_DATA_START = 6;
//    private static final int BLOCK_DATA_LENGTH = 7;
//    private static final int OFFSET_WITHIN_BLOCK_ID = 8;
//
//    // AprilTag ID与颜色序列的映射
//    private String[] colorSequence;
//
//    @Override
//    public void runOpMode() {
//        // 初始化硬件
//        initializeHardware();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // 检测AprilTag并获取颜色序列
//            detectAprilTagAndSetColors();
//
//            // 显示当前检测到的颜色序列
//            if (colorSequence != null) {
//                telemetry.addData("Color Sequence",
//                        colorSequence[0] + ", " + colorSequence[1] + ", " + colorSequence[2]);
//            } else {
//                telemetry.addData("Color Sequence", "No AprilTag detected");
//            }
//
//            // 原有的舵机测试代码可以保留在这里
//            // 例如根据游戏手柄输入控制舵机
//
//            telemetry.update();
//            sleep(100); // 降低检测频率，避免过度占用资源
//        }
//    }
//
//    private void initializeHardware() {
//        // 初始化舵机
//        testServo = hardwareMap.get(Servo.class, "testServo");
//
//        // 初始化二哈识图
//        try {
//            huskyLens = hardwareMap.get(I2cDeviceSynch.class, "huskylens");
//            huskyLens.setI2cAddress(HUSKYLENS_ADDRESS);
//            huskyLens.engage();
//            telemetry.addData("HuskyLens", "Initialized successfully");
//        } catch (Exception e) {
//            telemetry.addData("HuskyLens", "Initialization failed: " + e.getMessage());
//        }
//    }
//
//    private void detectAprilTagAndSetColors() {
//        try {
//            // 从二哈识图读取数据
//            byte[] readBuffer = huskyLens.read(REGISTER_READ_START, READ_LENGTH);
//
//            if (readBuffer != null && isValidHeader(readBuffer)) {
//                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;
//
//                if (blockCount > 0) {
//                    // 只处理第一个识别到的AprilTag
//                    int blockDataStart = OFFSET_BLOCK_DATA_START;
//                    int aprilTagId = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;
//
//                    telemetry.addData("Detected AprilTag ID", aprilTagId);
//
//                    // 根据AprilTag ID设置颜色序列
//                    setColorSequenceFromAprilTag(aprilTagId);
//                } else {
//                    colorSequence = null;
//                    telemetry.addData("AprilTag Detection", "No tags found");
//                }
//            } else {
//                colorSequence = null;
//                telemetry.addData("AprilTag Detection", "Invalid data or no data");
//            }
//        } catch (Exception e) {
//            colorSequence = null;
//            telemetry.addData("AprilTag Detection Error", e.getMessage());
//        }
//    }
//
//    private void setColorSequenceFromAprilTag(int aprilTagId) {
//        switch (aprilTagId) {
//            case 23:
//                colorSequence = new String[]{"颜色1", "颜色2", "颜色2"};
//                break;
//            case 22:
//                colorSequence = new String[]{"颜色2", "颜色1", "颜色2"};
//                break;
//            case 21:
//                colorSequence = new String[]{"颜色2", "颜色2", "颜色1"};
//                break;
//            default:
//                colorSequence = null;
//                telemetry.addData("Unknown AprilTag ID", aprilTagId);
//                break;
//        }
//    }
//
//    private boolean isValidHeader(byte[] buffer) {
//        // 根据二哈识图的实际通信协议实现帧头验证
//        // 这里需要根据二哈识图1的AprilTag识别协议进行调整
//        return buffer != null &&
//                buffer.length >= 2 &&
//                buffer[OFFSET_FRAME_HEADER] == (byte) 0x55 &&
//                buffer[OFFSET_FRAME_HEADER + 1] == (byte) 0xAA;
//    }
//
//    // 获取当前颜色序列的方法，供其他部分代码使用
//    public String[] getColorSequence() {
//        return colorSequence;
//    }
//}
//public class ServoTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotor motor = hardwareMap.get(DcMotor.class, "Motor");
//        Servo servo1 = hardwareMap.get(Servo.class,"servo1");
//
//        waitForStart();
//
////        while (opModeIsActive()) {
////            servo1.setPosition(0);
////
////        }
////
////        motor.setPower(0);
//    }
//}
