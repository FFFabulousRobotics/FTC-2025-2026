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

/**
 * ServoTest 类 - 使用二哈识图进行AprilTag识别和颜色区块识别
 * 并根据枚举算法控制舵机旋转
 *
 * 功能描述：
 * 1. 使用二哈识图1识别AprilTag并获取颜色序列
 * 2. 使用二哈识图2识别3个颜色区块并按从左到右排序
 * 3. 使用枚举算法根据AprilTag颜色序列和颜色区块匹配结果控制舵机
 * 4. 舵机旋转30度后会回归原位
 */
@TeleOp(name = "ServoTest with Enum Algorithm", group = "Test")
public class ServoTest extends LinearOpMode {

    // 定义三个舵机，分别对应三个颜色区块的位置
    private Servo servo1, servo2, servo3;

    // 定义两个二哈识图设备
    // huskyLens1 用于识别AprilTag
    // huskyLens2 用于识别颜色区块
    private I2cDeviceSynch huskyLens1, huskyLens2;

    // 二哈识图设备地址
    private static final I2cAddr HUSKYLENS1_ADDRESS = I2cAddr.create7bit(0x32); // AprilTag识别
    private static final I2cAddr HUSKYLENS2_ADDRESS = I2cAddr.create7bit(0x33); // 颜色识别

    // 数据读取相关常量
    private static final int REGISTER_READ_START = 0x00;
    private static final int READ_LENGTH = 36;

    // 数据帧解析偏移量（需要根据实际协议调整）
    private static final int OFFSET_FRAME_HEADER = 0;
    private static final int OFFSET_ALGORITHM_TYPE = 4;
    private static final int OFFSET_BLOCK_COUNT = 5;
    private static final int OFFSET_BLOCK_DATA_START = 6;
    private static final int BLOCK_DATA_LENGTH = 7;
    private static final int OFFSET_WITHIN_BLOCK_ID = 8;

    // 舵机角度常量（358度舵机，30度对应的位置值）
    private static final double SERVO_30_DEGREES = 30.0 / 358.0;
    private static final double SERVO_HOME_POSITION = 0.0; // 舵机初始位置

    // 颜色序列和区块列表
    private String[] aprilTagColorSequence; // 从AprilTag获取的颜色序列
    private List<ColorBlock> sortedColorBlocks; // 排序后的颜色区块列表

    // 舵机状态跟踪
    private boolean[] servoActivated = new boolean[3]; // 跟踪哪些舵机已被激活

    @Override
    public void runOpMode() {
        // 初始化所有硬件设备
        initializeHardware();

        telemetry.addData("状态", "初始化完成");
        telemetry.addData("说明", "按下START开始运行枚举算法");
        telemetry.update();

        // 等待用户按下START按钮
        waitForStart();

        // 主循环
        while (opModeIsActive()) {
            // 步骤1: 检测AprilTag并获取颜色序列
            detectAprilTagAndGetColorSequence();

            // 步骤2: 检测颜色区块并排序
            detectAndSortColorBlocks();

            // 步骤3: 使用枚举算法控制舵机
            if (aprilTagColorSequence != null && sortedColorBlocks.size() >= 3) {
                executeEnumerationAlgorithm();
            } else {
                telemetry.addData("错误", "未检测到足够的AprilTag或颜色区块");
            }

            // 显示当前状态信息
            displayStatusTelemetry();

            telemetry.update();
            sleep(500); // 适当延迟，避免过于频繁的检测
        }
    }

    /**
     * 初始化所有硬件设备
     * 包括三个舵机和两个二哈识图传感器
     */
    private void initializeHardware() {
        // 初始化三个舵机
        try {
            servo1 = hardwareMap.get(Servo.class, "servo1");
            servo2 = hardwareMap.get(Servo.class, "servo2");
            servo3 = hardwareMap.get(Servo.class, "servo3");

            // 设置舵机初始位置
            servo1.setPosition(SERVO_HOME_POSITION);
            servo2.setPosition(SERVO_HOME_POSITION);
            servo3.setPosition(SERVO_HOME_POSITION);

            telemetry.addData("舵机初始化", "成功");
        } catch (Exception e) {
            telemetry.addData("舵机初始化错误", e.getMessage());
        }

        // 初始化二哈识图1（用于AprilTag识别）
        try {
            huskyLens1 = hardwareMap.get(I2cDeviceSynch.class, "huskyLens1");
            huskyLens1.setI2cAddress(HUSKYLENS1_ADDRESS);
            huskyLens1.engage();
            telemetry.addData("二哈识图1初始化", "成功（AprilTag识别）");
        } catch (Exception e) {
            telemetry.addData("二哈识图1初始化错误", e.getMessage());
        }

        // 初始化二哈识图2（用于颜色识别）
        try {
            huskyLens2 = hardwareMap.get(I2cDeviceSynch.class, "huskyLens2");
            huskyLens2.setI2cAddress(HUSKYLENS2_ADDRESS);
            huskyLens2.engage();
            telemetry.addData("二哈识图2初始化", "成功（颜色识别）");
        } catch (Exception e) {
            telemetry.addData("二哈识图2初始化错误", e.getMessage());
        }

        // 初始化颜色区块列表
        sortedColorBlocks = new ArrayList<>();

        // 初始化舵机状态数组
        for (int i = 0; i < 3; i++) {
            servoActivated[i] = false;
        }
    }

    /**
     * 检测AprilTag并获取颜色序列
     * 根据AprilTag ID设置对应的颜色序列
     */
    private void detectAprilTagAndGetColorSequence() {
        try {
            // 从二哈识图1读取数据
            byte[] readBuffer = huskyLens1.read(REGISTER_READ_START, READ_LENGTH);

            if (readBuffer != null && isValidHeader(readBuffer)) {
                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;

                if (blockCount > 0) {
                    // 获取第一个识别到的AprilTag的ID
                    int blockDataStart = OFFSET_BLOCK_DATA_START;
                    int aprilTagId = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;

                    telemetry.addData("检测到的AprilTag ID", aprilTagId);

                    // 根据AprilTag ID设置颜色序列
                    setColorSequenceFromAprilTag(aprilTagId);
                } else {
                    aprilTagColorSequence = null;
                    telemetry.addData("AprilTag检测", "未检测到AprilTag");
                }
            } else {
                aprilTagColorSequence = null;
                telemetry.addData("AprilTag检测", "数据无效或无数据");
            }
        } catch (Exception e) {
            aprilTagColorSequence = null;
            telemetry.addData("AprilTag检测错误", e.getMessage());
        }
    }

    /**
     * 检测颜色区块并按从左到右排序
     * 区块的X坐标越小，位置越靠左
     */
    private void detectAndSortColorBlocks() {
        sortedColorBlocks.clear();

        try {
            // 从二哈识图2读取数据
            byte[] readBuffer = huskyLens2.read(REGISTER_READ_START, READ_LENGTH);

            if (readBuffer != null && isValidHeader(readBuffer)) {
                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;

                // 处理所有检测到的颜色区块（最多3个）
                for (int i = 0; i < blockCount && i < 3; i++) {
                    int blockDataStart = OFFSET_BLOCK_DATA_START + i * BLOCK_DATA_LENGTH;

                    if (blockDataStart + BLOCK_DATA_LENGTH <= readBuffer.length) {
                        // 解析区块数据
                        int xCenter = ((readBuffer[blockDataStart] & 0xFF) << 8) |
                                (readBuffer[blockDataStart + 1] & 0xFF);
                        int yCenter = ((readBuffer[blockDataStart + 2] & 0xFF) << 8) |
                                (readBuffer[blockDataStart + 3] & 0xFF);
                        int width = ((readBuffer[blockDataStart + 4] & 0xFF) << 8) |
                                (readBuffer[blockDataStart + 5] & 0xFF);
                        int height = ((readBuffer[blockDataStart + 6] & 0xFF) << 8) |
                                (readBuffer[blockDataStart + 7] & 0xFF);
                        int id = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;

                        // 根据ID确定颜色（需要根据实际学习设置调整）
                        String color = (id == 1) ? "颜色1" : "颜色2";

                        // 创建颜色区块对象并添加到列表
                        sortedColorBlocks.add(new ColorBlock(color, xCenter, yCenter, width, height));
                    }
                }

                // 按X坐标排序（从左到右）
                Collections.sort(sortedColorBlocks, new Comparator<ColorBlock>() {
                    @Override
                    public int compare(ColorBlock block1, ColorBlock block2) {
                        return Integer.compare(block1.xCenter, block2.xCenter);
                    }
                });

            } else {
                telemetry.addData("颜色检测", "数据无效或无数据");
            }
        } catch (Exception e) {
            telemetry.addData("颜色检测错误", e.getMessage());
        }
    }

    /**
     * 执行枚举算法控制舵机
     * 算法逻辑：
     * 1. 对于AprilTag颜色序列中的每个颜色
     * 2. 按顺序检查颜色区块是否匹配
     * 3. 如果匹配，则旋转对应的舵机30度，然后回归原位
     * 4. 如果所有区块都不匹配，则不进行任何操作
     */
    private void executeEnumerationAlgorithm() {
        // 重置舵机状态
        resetServoStates();

        // 第一轮比较：AprilTag第一个颜色 vs 颜色区块
        telemetry.addLine("=== 第一轮比较 ===");
        boolean firstColorMatched = compareColorAndActivateServo(0, 0, 1, 2);

        // 第二轮比较：AprilTag第二个颜色 vs 颜色区块
        telemetry.addLine("=== 第二轮比较 ===");
        boolean secondColorMatched = compareColorAndActivateServo(1, 0, 1, 2);

        // 第三轮比较：AprilTag第三个颜色 vs 颜色区块
        telemetry.addLine("=== 第三轮比较 ===");
        boolean thirdColorMatched = compareColorAndActivateServo(2, 0, 1, 2);

        // 显示匹配结果
        telemetry.addData("第一轮匹配", firstColorMatched ? "成功" : "失败");
        telemetry.addData("第二轮匹配", secondColorMatched ? "成功" : "失败");
        telemetry.addData("第三轮匹配", thirdColorMatched ? "成功" : "失败");
    }

    /**
     * 比较颜色并激活舵机
     * @param aprilTagIndex AprilTag颜色序列中的索引
     * @param block1Index 第一个颜色区块索引
     * @param block2Index 第二个颜色区块索引
     * @param block3Index 第三个颜色区块索引
     * @return 是否找到匹配并激活了舵机
     */
    private boolean compareColorAndActivateServo(int aprilTagIndex, int block1Index, int block2Index, int block3Index) {
        String targetColor = aprilTagColorSequence[aprilTagIndex];

        // 检查第一个颜色区块
        if (block1Index < sortedColorBlocks.size() &&
                sortedColorBlocks.get(block1Index).color.equals(targetColor) &&
                !servoActivated[0]) {

            telemetry.addData("匹配", "AprilTag颜色" + (aprilTagIndex+1) + " 与 区块" + (block1Index+1) + " 匹配");
            activateServo(0); // 激活1号舵机
            return true;
        }

        // 检查第二个颜色区块
        if (block2Index < sortedColorBlocks.size() &&
                sortedColorBlocks.get(block2Index).color.equals(targetColor) &&
                !servoActivated[1]) {

            telemetry.addData("匹配", "AprilTag颜色" + (aprilTagIndex+1) + " 与 区块" + (block2Index+1) + " 匹配");
            activateServo(1); // 激活2号舵机
            return true;
        }

        // 检查第三个颜色区块
        if (block3Index < sortedColorBlocks.size() &&
                sortedColorBlocks.get(block3Index).color.equals(targetColor) &&
                !servoActivated[2]) {

            telemetry.addData("匹配", "AprilTag颜色" + (aprilTagIndex+1) + " 与 区块" + (block3Index+1) + " 匹配");
            activateServo(2); // 激活3号舵机
            return true;
        }

        telemetry.addData("匹配", "AprilTag颜色" + (aprilTagIndex+1) + " 未找到匹配区块");
        return false;
    }

    /**
     * 激活指定舵机：旋转30度后回归原位
     * @param servoIndex 舵机索引（0,1,2 对应 舵机1,2,3）
     */
    private void activateServo(int servoIndex) {
        Servo targetServo = null;
        String servoName = "";

        // 选择目标舵机
        switch (servoIndex) {
            case 0:
                targetServo = servo1;
                servoName = "舵机1";
                break;
            case 1:
                targetServo = servo2;
                servoName = "舵机2";
                break;
            case 2:
                targetServo = servo3;
                servoName = "舵机3";
                break;
        }

        if (targetServo != null) {
            try {
                // 旋转舵机30度
                targetServo.setPosition(SERVO_30_DEGREES);
                telemetry.addData("舵机动作", servoName + " 旋转30度");
                sleep(500); // 等待舵机完成旋转

                // 回归原位
                targetServo.setPosition(SERVO_HOME_POSITION);
                telemetry.addData("舵机动作", servoName + " 回归原位");
                sleep(300); // 等待舵机回归原位

                // 标记舵机已激活
                servoActivated[servoIndex] = true;

            } catch (Exception e) {
                telemetry.addData("舵机控制错误", servoName + ": " + e.getMessage());
            }
        }
    }

    /**
     * 重置舵机状态
     * 在每次执行枚举算法前调用
     */
    private void resetServoStates() {
        for (int i = 0; i < 3; i++) {
            servoActivated[i] = false;
        }

        // 确保所有舵机都在初始位置
        servo1.setPosition(SERVO_HOME_POSITION);
        servo2.setPosition(SERVO_HOME_POSITION);
        servo3.setPosition(SERVO_HOME_POSITION);
    }

    /**
     * 根据AprilTag ID设置颜色序列
     * 修改后的映射关系：
     * ID=1: 颜色序列为 ["颜色1", "颜色2", "颜色2"]
     * ID=2: 颜色序列为 ["颜色2", "颜色1", "颜色2"]
     * ID=3: 颜色序列为 ["颜色2", "颜色2", "颜色1"]
     *
     * @param aprilTagId 检测到的AprilTag ID
     */
    private void setColorSequenceFromAprilTag(int aprilTagId) {
        switch (aprilTagId) {
            case 1:
                aprilTagColorSequence = new String[]{"颜色1", "颜色2", "颜色2"};
                telemetry.addData("颜色序列", "AprilTag 1: 颜色1, 颜色2, 颜色2");
                break;
            case 2:
                aprilTagColorSequence = new String[]{"颜色2", "颜色1", "颜色2"};
                telemetry.addData("颜色序列", "AprilTag 2: 颜色2, 颜色1, 颜色2");
                break;
            case 3:
                aprilTagColorSequence = new String[]{"颜色2", "颜色2", "颜色1"};
                telemetry.addData("颜色序列", "AprilTag 3: 颜色2, 颜色2, 颜色1");
                break;
            default:
                aprilTagColorSequence = null;
                telemetry.addData("未知AprilTag ID", aprilTagId);
                break;
        }
    }

    /**
     * 显示状态信息到Telemetry
     */
    private void displayStatusTelemetry() {
        // 显示AprilTag信息
        if (aprilTagColorSequence != null) {
            telemetry.addLine("=== AprilTag信息 ===");
            telemetry.addData("颜色序列",
                    aprilTagColorSequence[0] + ", " +
                            aprilTagColorSequence[1] + ", " +
                            aprilTagColorSequence[2]);
        } else {
            telemetry.addData("AprilTag", "未检测到或识别失败");
        }

        // 显示颜色区块信息
        telemetry.addLine("=== 颜色区块信息 ===");
        if (sortedColorBlocks.size() > 0) {
            telemetry.addData("检测到的区块数量", sortedColorBlocks.size());
            for (int i = 0; i < sortedColorBlocks.size(); i++) {
                ColorBlock block = sortedColorBlocks.get(i);
                telemetry.addData("区块 " + (i+1),
                        block.color + " (X:" + block.xCenter + ", Y:" + block.yCenter + ")");
            }
        } else {
            telemetry.addData("颜色区块", "未检测到颜色区块");
        }

        // 显示舵机状态
        telemetry.addLine("=== 舵机状态 ===");
        for (int i = 0; i < 3; i++) {
            telemetry.addData("舵机 " + (i+1), servoActivated[i] ? "已激活" : "未激活");
        }
    }

    /**
     * 验证数据帧头是否有效
     * @param buffer 数据缓冲区
     * @return 帧头是否有效
     */
    private boolean isValidHeader(byte[] buffer) {
        return buffer != null &&
                buffer.length >= 2 &&
                buffer[OFFSET_FRAME_HEADER] == (byte) 0x55 &&
                buffer[OFFSET_FRAME_HEADER + 1] == (byte) 0xAA;
    }

    /**
     * 颜色区块内部类
     * 用于存储颜色区块的相关信息
     */
    private class ColorBlock {
        String color;      // 区块颜色
        int xCenter;       // 区块中心X坐标
        int yCenter;       // 区块中心Y坐标
        int width;         // 区块宽度
        int height;        // 区块高度

        public ColorBlock(String color, int xCenter, int yCenter, int width, int height) {
            this.color = color;
            this.xCenter = xCenter;
            this.yCenter = yCenter;
            this.width = width;
            this.height = height;
        }
    }
}
//package org.firstinspires.ftc.teamcode.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cAddr;
//
//import java.util.ArrayList;
//import java.util.Collections;
//import java.util.Comparator;
//import java.util.List;
//
///**
// * ServoTest 类 - 使用二哈识图进行AprilTag识别和颜色区块识别
// * 并根据枚举算法控制舵机旋转
// *
// * 功能描述：
// * 1. 使用二哈识图1识别AprilTag并获取颜色序列
// * 2. 使用二哈识图2识别3个颜色区块并按从左到右排序
// * 3. 使用枚举算法根据AprilTag颜色序列和颜色区块匹配结果控制舵机
// * 4. 舵机旋转30度后会回归原位
// */
//@TeleOp(name = "ServoTest with Enum Algorithm", group = "Test")
//public class ServoTest extends LinearOpMode {
//
//    // 定义三个舵机，分别对应三个颜色区块的位置
//    private Servo servo1, servo2, servo3;
//
//    // 定义两个二哈识图设备
//    // huskyLens1 用于识别AprilTag
//    // huskyLens2 用于识别颜色区块
//    private I2cDeviceSynch huskyLens1, huskyLens2;
//
//    // 二哈识图设备地址
//    private static final I2cAddr HUSKYLENS1_ADDRESS = I2cAddr.create7bit(0x32); // AprilTag识别
//    private static final I2cAddr HUSKYLENS2_ADDRESS = I2cAddr.create7bit(0x33); // 颜色识别
//
//    // 数据读取相关常量
//    private static final int REGISTER_READ_START = 0x00;
//    private static final int READ_LENGTH = 36;
//
//    // 数据帧解析偏移量（需要根据实际协议调整）
//    private static final int OFFSET_FRAME_HEADER = 0;
//    private static final int OFFSET_ALGORITHM_TYPE = 4;
//    private static final int OFFSET_BLOCK_COUNT = 5;
//    private static final int OFFSET_BLOCK_DATA_START = 6;
//    private static final int BLOCK_DATA_LENGTH = 7;
//    private static final int OFFSET_WITHIN_BLOCK_ID = 8;
//
//    // 舵机角度常量（358度舵机，30度对应的位置值）
//    private static final double SERVO_30_DEGREES = 30.0 / 358.0;
//    private static final double SERVO_HOME_POSITION = 0.0; // 舵机初始位置
//
//    // 颜色序列和区块列表
//    private String[] aprilTagColorSequence; // 从AprilTag获取的颜色序列
//    private List<ColorBlock> sortedColorBlocks; // 排序后的颜色区块列表
//
//    // 舵机状态跟踪
//    private boolean[] servoActivated = new boolean[3]; // 跟踪哪些舵机已被激活
//
//    @Override
//    public void runOpMode() {
//        // 初始化所有硬件设备
//        initializeHardware();
//
//        telemetry.addData("状态", "初始化完成");
//        telemetry.addData("说明", "按下START开始运行枚举算法");
//        telemetry.update();
//
//        // 等待用户按下START按钮
//        waitForStart();
//
//        // 主循环
//        while (opModeIsActive()) {
//            // 步骤1: 检测AprilTag并获取颜色序列
//            detectAprilTagAndGetColorSequence();
//
//            // 步骤2: 检测颜色区块并排序
//            detectAndSortColorBlocks();
//
//            // 步骤3: 使用枚举算法控制舵机
//            if (aprilTagColorSequence != null && sortedColorBlocks.size() >= 3) {
//                executeEnumerationAlgorithm();
//            } else {
//                telemetry.addData("错误", "未检测到足够的AprilTag或颜色区块");
//            }
//
//            // 显示当前状态信息
//            displayStatusTelemetry();
//
//            telemetry.update();
//            sleep(500); // 适当延迟，避免过于频繁的检测
//        }
//    }
//
//    /**
//     * 初始化所有硬件设备
//     * 包括三个舵机和两个二哈识图传感器
//     */
//    private void initializeHardware() {
//        // 初始化三个舵机
//        try {
//            servo1 = hardwareMap.get(Servo.class, "servo1");
//            servo2 = hardwareMap.get(Servo.class, "servo2");
//            servo3 = hardwareMap.get(Servo.class, "servo3");
//
//            // 设置舵机初始位置
//            servo1.setPosition(SERVO_HOME_POSITION);
//            servo2.setPosition(SERVO_HOME_POSITION);
//            servo3.setPosition(SERVO_HOME_POSITION);
//
//            telemetry.addData("舵机初始化", "成功");
//        } catch (Exception e) {
//            telemetry.addData("舵机初始化错误", e.getMessage());
//        }
//
//        // 初始化二哈识图1（用于AprilTag识别）
//        try {
//            huskyLens1 = hardwareMap.get(I2cDeviceSynch.class, "huskyLens1");
//            huskyLens1.setI2cAddress(HUSKYLENS1_ADDRESS);
//            huskyLens1.engage();
//            telemetry.addData("二哈识图1初始化", "成功（AprilTag识别）");
//        } catch (Exception e) {
//            telemetry.addData("二哈识图1初始化错误", e.getMessage());
//        }
//
//        // 初始化二哈识图2（用于颜色识别）
//        try {
//            huskyLens2 = hardwareMap.get(I2cDeviceSynch.class, "huskyLens2");
//            huskyLens2.setI2cAddress(HUSKYLENS2_ADDRESS);
//            huskyLens2.engage();
//            telemetry.addData("二哈识图2初始化", "成功（颜色识别）");
//        } catch (Exception e) {
//            telemetry.addData("二哈识图2初始化错误", e.getMessage());
//        }
//
//        // 初始化颜色区块列表
//        sortedColorBlocks = new ArrayList<>();
//
//        // 初始化舵机状态数组
//        for (int i = 0; i < 3; i++) {
//            servoActivated[i] = false;
//        }
//    }
//
//    /**
//     * 检测AprilTag并获取颜色序列
//     * 根据AprilTag ID设置对应的颜色序列
//     */
//    private void detectAprilTagAndGetColorSequence() {
//        try {
//            // 从二哈识图1读取数据
//            byte[] readBuffer = huskyLens1.read(REGISTER_READ_START, READ_LENGTH);
//
//            if (readBuffer != null && isValidHeader(readBuffer)) {
//                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;
//
//                if (blockCount > 0) {
//                    // 获取第一个识别到的AprilTag的ID
//                    int blockDataStart = OFFSET_BLOCK_DATA_START;
//                    int aprilTagId = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;
//
//                    telemetry.addData("检测到的AprilTag ID", aprilTagId);
//
//                    // 根据AprilTag ID设置颜色序列
//                    setColorSequenceFromAprilTag(aprilTagId);
//                } else {
//                    aprilTagColorSequence = null;
//                    telemetry.addData("AprilTag检测", "未检测到AprilTag");
//                }
//            } else {
//                aprilTagColorSequence = null;
//                telemetry.addData("AprilTag检测", "数据无效或无数据");
//            }
//        } catch (Exception e) {
//            aprilTagColorSequence = null;
//            telemetry.addData("AprilTag检测错误", e.getMessage());
//        }
//    }
//
//    /**
//     * 检测颜色区块并按从左到右排序
//     * 区块的X坐标越小，位置越靠左
//     */
//    private void detectAndSortColorBlocks() {
//        sortedColorBlocks.clear();
//
//        try {
//            // 从二哈识图2读取数据
//            byte[] readBuffer = huskyLens2.read(REGISTER_READ_START, READ_LENGTH);
//
//            if (readBuffer != null && isValidHeader(readBuffer)) {
//                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;
//
//                // 处理所有检测到的颜色区块（最多3个）
//                for (int i = 0; i < blockCount && i < 3; i++) {
//                    int blockDataStart = OFFSET_BLOCK_DATA_START + i * BLOCK_DATA_LENGTH;
//
//                    if (blockDataStart + BLOCK_DATA_LENGTH <= readBuffer.length) {
//                        // 解析区块数据
//                        int xCenter = ((readBuffer[blockDataStart] & 0xFF) << 8) |
//                                (readBuffer[blockDataStart + 1] & 0xFF);
//                        int yCenter = ((readBuffer[blockDataStart + 2] & 0xFF) << 8) |
//                                (readBuffer[blockDataStart + 3] & 0xFF);
//                        int width = ((readBuffer[blockDataStart + 4] & 0xFF) << 8) |
//                                (readBuffer[blockDataStart + 5] & 0xFF);
//                        int height = ((readBuffer[blockDataStart + 6] & 0xFF) << 8) |
//                                (readBuffer[blockDataStart + 7] & 0xFF);
//                        int id = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;
//
//                        // 根据ID确定颜色（需要根据实际学习设置调整）
//                        String color = (id == 1) ? "颜色1" : "颜色2";
//
//                        // 创建颜色区块对象并添加到列表
//                        sortedColorBlocks.add(new ColorBlock(color, xCenter, yCenter, width, height));
//                    }
//                }
//
//                // 按X坐标排序（从左到右）
//                Collections.sort(sortedColorBlocks, new Comparator<ColorBlock>() {
//                    @Override
//                    public int compare(ColorBlock block1, ColorBlock block2) {
//                        return Integer.compare(block1.xCenter, block2.xCenter);
//                    }
//                });
//
//            } else {
//                telemetry.addData("颜色检测", "数据无效或无数据");
//            }
//        } catch (Exception e) {
//            telemetry.addData("颜色检测错误", e.getMessage());
//        }
//    }
//
//    /**
//     * 执行枚举算法控制舵机
//     * 算法逻辑：
//     * 1. 对于AprilTag颜色序列中的每个颜色
//     * 2. 按顺序检查颜色区块是否匹配
//     * 3. 如果匹配，则旋转对应的舵机30度，然后回归原位
//     * 4. 如果所有区块都不匹配，则不进行任何操作
//     */
//    private void executeEnumerationAlgorithm() {
//        // 重置舵机状态
//        resetServoStates();
//
//        // 第一轮比较：AprilTag第一个颜色 vs 颜色区块
//        telemetry.addLine("=== 第一轮比较 ===");
//        boolean firstColorMatched = compareColorAndActivateServo(0, 0, 1, 2);
//
//        // 第二轮比较：AprilTag第二个颜色 vs 颜色区块
//        telemetry.addLine("=== 第二轮比较 ===");
//        boolean secondColorMatched = compareColorAndActivateServo(1, 0, 1, 2);
//
//        // 第三轮比较：AprilTag第三个颜色 vs 颜色区块
//        telemetry.addLine("=== 第三轮比较 ===");
//        boolean thirdColorMatched = compareColorAndActivateServo(2, 0, 1, 2);
//
//        // 显示匹配结果
//        telemetry.addData("第一轮匹配", firstColorMatched ? "成功" : "失败");
//        telemetry.addData("第二轮匹配", secondColorMatched ? "成功" : "失败");
//        telemetry.addData("第三轮匹配", thirdColorMatched ? "成功" : "失败");
//    }
//
//    /**
//     * 比较颜色并激活舵机
//     * @param aprilTagIndex AprilTag颜色序列中的索引
//     * @param block1Index 第一个颜色区块索引
//     * @param block2Index 第二个颜色区块索引
//     * @param block3Index 第三个颜色区块索引
//     * @return 是否找到匹配并激活了舵机
//     */
//    private boolean compareColorAndActivateServo(int aprilTagIndex, int block1Index, int block2Index, int block3Index) {
//        String targetColor = aprilTagColorSequence[aprilTagIndex];
//
//        // 检查第一个颜色区块
//        if (block1Index < sortedColorBlocks.size() &&
//                sortedColorBlocks.get(block1Index).color.equals(targetColor) &&
//                !servoActivated[0]) {
//
//            telemetry.addData("匹配", "AprilTag颜色" + (aprilTagIndex+1) + " 与 区块" + (block1Index+1) + " 匹配");
//            activateServo(0); // 激活1号舵机
//            return true;
//        }
//
//        // 检查第二个颜色区块
//        if (block2Index < sortedColorBlocks.size() &&
//                sortedColorBlocks.get(block2Index).color.equals(targetColor) &&
//                !servoActivated[1]) {
//
//            telemetry.addData("匹配", "AprilTag颜色" + (aprilTagIndex+1) + " 与 区块" + (block2Index+1) + " 匹配");
//            activateServo(1); // 激活2号舵机
//            return true;
//        }
//
//        // 检查第三个颜色区块
//        if (block3Index < sortedColorBlocks.size() &&
//                sortedColorBlocks.get(block3Index).color.equals(targetColor) &&
//                !servoActivated[2]) {
//
//            telemetry.addData("匹配", "AprilTag颜色" + (aprilTagIndex+1) + " 与 区块" + (block3Index+1) + " 匹配");
//            activateServo(2); // 激活3号舵机
//            return true;
//        }
//
//        telemetry.addData("匹配", "AprilTag颜色" + (aprilTagIndex+1) + " 未找到匹配区块");
//        return false;
//    }
//
//    /**
//     * 激活指定舵机：旋转30度后回归原位
//     * @param servoIndex 舵机索引（0,1,2 对应 舵机1,2,3）
//     */
//    private void activateServo(int servoIndex) {
//        Servo targetServo = null;
//        String servoName = "";
//
//        // 选择目标舵机
//        switch (servoIndex) {
//            case 0:
//                targetServo = servo1;
//                servoName = "舵机1";
//                break;
//            case 1:
//                targetServo = servo2;
//                servoName = "舵机2";
//                break;
//            case 2:
//                targetServo = servo3;
//                servoName = "舵机3";
//                break;
//        }
//
//        if (targetServo != null) {
//            try {
//                // 旋转舵机30度
//                targetServo.setPosition(SERVO_30_DEGREES);
//                telemetry.addData("舵机动作", servoName + " 旋转30度");
//                sleep(500); // 等待舵机完成旋转
//
//                // 回归原位
//                targetServo.setPosition(SERVO_HOME_POSITION);
//                telemetry.addData("舵机动作", servoName + " 回归原位");
//                sleep(300); // 等待舵机回归原位
//
//                // 标记舵机已激活
//                servoActivated[servoIndex] = true;
//
//            } catch (Exception e) {
//                telemetry.addData("舵机控制错误", servoName + ": " + e.getMessage());
//            }
//        }
//    }
//
//    /**
//     * 重置舵机状态
//     * 在每次执行枚举算法前调用
//     */
//    private void resetServoStates() {
//        for (int i = 0; i < 3; i++) {
//            servoActivated[i] = false;
//        }
//
//        // 确保所有舵机都在初始位置
//        servo1.setPosition(SERVO_HOME_POSITION);
//        servo2.setPosition(SERVO_HOME_POSITION);
//        servo3.setPosition(SERVO_HOME_POSITION);
//    }
//
//    /**
//     * 根据AprilTag ID设置颜色序列
//     * @param aprilTagId 检测到的AprilTag ID
//     */
//    private void setColorSequenceFromAprilTag(int aprilTagId) {
//        switch (aprilTagId) {
//            case 23:
//                aprilTagColorSequence = new String[]{"颜色1", "颜色2", "颜色2"};
//                telemetry.addData("颜色序列", "AprilTag 23: 颜色1, 颜色2, 颜色2");
//                break;
//            case 22:
//                aprilTagColorSequence = new String[]{"颜色2", "颜色1", "颜色2"};
//                telemetry.addData("颜色序列", "AprilTag 22: 颜色2, 颜色1, 颜色2");
//                break;
//            case 21:
//                aprilTagColorSequence = new String[]{"颜色2", "颜色2", "颜色1"};
//                telemetry.addData("颜色序列", "AprilTag 21: 颜色2, 颜色2, 颜色1");
//                break;
//            default:
//                aprilTagColorSequence = null;
//                telemetry.addData("未知AprilTag ID", aprilTagId);
//                break;
//        }
//    }
//
//    /**
//     * 显示状态信息到Telemetry
//     */
//    private void displayStatusTelemetry() {
//        // 显示AprilTag信息
//        if (aprilTagColorSequence != null) {
//            telemetry.addLine("=== AprilTag信息 ===");
//            telemetry.addData("颜色序列",
//                    aprilTagColorSequence[0] + ", " +
//                            aprilTagColorSequence[1] + ", " +
//                            aprilTagColorSequence[2]);
//        } else {
//            telemetry.addData("AprilTag", "未检测到或识别失败");
//        }
//
//        // 显示颜色区块信息
//        telemetry.addLine("=== 颜色区块信息 ===");
//        if (sortedColorBlocks.size() > 0) {
//            telemetry.addData("检测到的区块数量", sortedColorBlocks.size());
//            for (int i = 0; i < sortedColorBlocks.size(); i++) {
//                ColorBlock block = sortedColorBlocks.get(i);
//                telemetry.addData("区块 " + (i+1),
//                        block.color + " (X:" + block.xCenter + ", Y:" + block.yCenter + ")");
//            }
//        } else {
//            telemetry.addData("颜色区块", "未检测到颜色区块");
//        }
//
//        // 显示舵机状态
//        telemetry.addLine("=== 舵机状态 ===");
//        for (int i = 0; i < 3; i++) {
//            telemetry.addData("舵机 " + (i+1), servoActivated[i] ? "已激活" : "未激活");
//        }
//    }
//
//    /**
//     * 验证数据帧头是否有效
//     * @param buffer 数据缓冲区
//     * @return 帧头是否有效
//     */
//    private boolean isValidHeader(byte[] buffer) {
//        return buffer != null &&
//                buffer.length >= 2 &&
//                buffer[OFFSET_FRAME_HEADER] == (byte) 0x55 &&
//                buffer[OFFSET_FRAME_HEADER + 1] == (byte) 0xAA;
//    }
//
//    /**
//     * 颜色区块内部类
//     * 用于存储颜色区块的相关信息
//     */
//    private class ColorBlock {
//        String color;      // 区块颜色
//        int xCenter;       // 区块中心X坐标
//        int yCenter;       // 区块中心Y坐标
//        int width;         // 区块宽度
//        int height;        // 区块高度
//
//        public ColorBlock(String color, int xCenter, int yCenter, int width, int height) {
//            this.color = color;
//            this.xCenter = xCenter;
//            this.yCenter = yCenter;
//            this.width = width;
//            this.height = height;
//        }
//    }
//}

//package org.firstinspires.ftc.teamcode.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cAddr;
//
//import java.util.ArrayList;
//import java.util.Collections;
//import java.util.Comparator;
//import java.util.List;
//
//@TeleOp(name = "ServoTest with HuskyLens", group = "Test")
//public class ServoTest extends LinearOpMode {
//
//    private Servo testServo1, testServo2, testServo3; // 三个舵机
//    private I2cDeviceSynch huskyLens1; // 二哈识图1用于AprilTag识别
//    private I2cDeviceSynch huskyLens2; // 二哈识图2用于颜色识别
//
//    // 二哈识图1相关常量 (AprilTag识别)
//    private static final I2cAddr HUSKYLENS1_ADDRESS = I2cAddr.create7bit(0x32);
//
//    // 二哈识图2相关常量 (颜色识别)
//    private static final I2cAddr HUSKYLENS2_ADDRESS = I2cAddr.create7bit(0x33);
//
//    private static final int REGISTER_READ_START = 0x00;
//    private static final int READ_LENGTH = 36;
//
//    // AprilTag识别结果的数据偏移量
//    private static final int OFFSET_FRAME_HEADER = 0;
//    private static final int OFFSET_ALGORITHM_TYPE = 4;
//    private static final int OFFSET_BLOCK_COUNT = 5;
//    private static final int OFFSET_BLOCK_DATA_START = 6;
//    private static final int BLOCK_DATA_LENGTH = 7;
//    private static final int OFFSET_WITHIN_BLOCK_ID = 8;
//
//    // 颜色区块信息
//    private List<ColorBlock> colorBlocks;
//    private String[] colorSequence;
//
//    // 舵机角度常量 (358度舵机，30度对应的位置值)
//    private static final double SERVO_30_DEGREES = 30.0 / 358.0;
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
//            // 1. 检测AprilTag并获取颜色序列
//            detectAprilTagAndSetColors();
//
//            // 2. 检测颜色区块并排序
//            detectAndSortColorBlocks();
//
//            // 3. 根据颜色序列控制舵机
//            controlServosBasedOnColorSequence();
//
//            // 显示当前状态
//            displayTelemetry();
//
//            telemetry.update();
//            sleep(100); // 降低检测频率
//        }
//    }
//
//    private void initializeHardware() {
//        // 初始化三个舵机
//        testServo1 = hardwareMap.get(Servo.class, "testServo1");
//        testServo2 = hardwareMap.get(Servo.class, "testServo2");
//        testServo3 = hardwareMap.get(Servo.class, "testServo3");
//
//        // 初始化二哈识图1 (AprilTag识别)
//        try {
//            huskyLens1 = hardwareMap.get(I2cDeviceSynch.class, "huskylens1");
//            huskyLens1.setI2cAddress(HUSKYLENS1_ADDRESS);
//            huskyLens1.engage();
//            telemetry.addData("HuskyLens1", "Initialized successfully");
//        } catch (Exception e) {
//            telemetry.addData("HuskyLens1", "Initialization failed: " + e.getMessage());
//        }
//
//        // 初始化二哈识图2 (颜色识别)
//        try {
//            huskyLens2 = hardwareMap.get(I2cDeviceSynch.class, "huskylens2");
//            huskyLens2.setI2cAddress(HUSKYLENS2_ADDRESS);
//            huskyLens2.engage();
//            telemetry.addData("HuskyLens2", "Initialized successfully");
//        } catch (Exception e) {
//            telemetry.addData("HuskyLens2", "Initialization failed: " + e.getMessage());
//        }
//
//        colorBlocks = new ArrayList<>();
//    }
//
//    private void detectAprilTagAndSetColors() {
//        try {
//            byte[] readBuffer = huskyLens1.read(REGISTER_READ_START, READ_LENGTH);
//
//            if (readBuffer != null && isValidHeader(readBuffer)) {
//                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;
//
//                if (blockCount > 0) {
//                    int blockDataStart = OFFSET_BLOCK_DATA_START;
//                    int aprilTagId = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;
//
//                    telemetry.addData("Detected AprilTag ID", aprilTagId);
//
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
//    private void detectAndSortColorBlocks() {
//        colorBlocks.clear();
//
//        try {
//            byte[] readBuffer = huskyLens2.read(REGISTER_READ_START, READ_LENGTH);
//
//            if (readBuffer != null && isValidHeader(readBuffer)) {
//                int blockCount = readBuffer[OFFSET_BLOCK_COUNT] & 0xFF;
//
//                for (int i = 0; i < blockCount && i < 3; i++) { // 最多处理3个区块
//                    int blockDataStart = OFFSET_BLOCK_DATA_START + i * BLOCK_DATA_LENGTH;
//
//                    if (blockDataStart + BLOCK_DATA_LENGTH <= readBuffer.length) {
//                        int xCenter = ((readBuffer[blockDataStart] & 0xFF) << 8) | (readBuffer[blockDataStart + 1] & 0xFF);
//                        int yCenter = ((readBuffer[blockDataStart + 2] & 0xFF) << 8) | (readBuffer[blockDataStart + 3] & 0xFF);
//                        int width = ((readBuffer[blockDataStart + 4] & 0xFF) << 8) | (readBuffer[blockDataStart + 5] & 0xFF);
//                        int height = ((readBuffer[blockDataStart + 6] & 0xFF) << 8) | (readBuffer[blockDataStart + 7] & 0xFF);
//                        int id = readBuffer[blockDataStart + OFFSET_WITHIN_BLOCK_ID] & 0xFF;
//
//                        // 根据ID判断颜色 (需要根据你的颜色学习设置调整)
//                        String color = (id == 1) ? "颜色1" : "颜色2";
//
//                        colorBlocks.add(new ColorBlock(color, xCenter, yCenter, width, height, i + 1));
//                    }
//                }
//
//                // 按X坐标排序 (从左到右)
//                Collections.sort(colorBlocks, new Comparator<ColorBlock>() {
//                    @Override
//                    public int compare(ColorBlock b1, ColorBlock b2) {
//                        return Integer.compare(b1.xCenter, b2.xCenter);
//                    }
//                });
//
//                // 重新编号 (1, 2, 3)
//                for (int i = 0; i < colorBlocks.size(); i++) {
//                    colorBlocks.get(i).position = i + 1;
//                }
//
//            } else {
//                telemetry.addData("Color Detection", "Invalid data or no data");
//            }
//        } catch (Exception e) {
//            telemetry.addData("Color Detection Error", e.getMessage());
//        }
//    }
//
//    private void controlServosBasedOnColorSequence() {
//        if (colorSequence == null || colorBlocks.size() < 3) {
//            telemetry.addData("Servo Control", "Waiting for AprilTag and 3 color blocks...");
//            return;
//        }
//
//        // 重置所有舵机位置
//        testServo1.setPosition(0);
//        testServo2.setPosition(0);
//        testServo3.setPosition(0);
//
//        // 根据颜色序列控制对应的舵机
//        for (int i = 0; i < 3; i++) {
//            String expectedColor = colorSequence[i];
//
//            // 找到对应位置的区块
//            for (ColorBlock block : colorBlocks) {
//                if (block.position == i + 1 && block.color.equals(expectedColor)) {
//                    // 根据位置选择舵机并旋转30度
//                    switch (i) {
//                        case 0: // 最左边
//                            testServo1.setPosition(SERVO_30_DEGREES);
//                            telemetry.addData("Servo1", "Rotated 30° for " + expectedColor);
//                            break;
//                        case 1: // 中间
//                            testServo2.setPosition(SERVO_30_DEGREES);
//                            telemetry.addData("Servo2", "Rotated 30° for " + expectedColor);
//                            break;
//                        case 2: // 最右边
//                            testServo3.setPosition(SERVO_30_DEGREES);
//                            telemetry.addData("Servo3", "Rotated 30° for " + expectedColor);
//                            break;
//                    }
//                    break;
//                }
//            }
//        }
//    }
//
//    private void displayTelemetry() {
//        if (colorSequence != null) {
//            telemetry.addData("AprilTag Color Sequence",
//                    colorSequence[0] + ", " + colorSequence[1] + ", " + colorSequence[2]);
//        } else {
//            telemetry.addData("AprilTag Color Sequence", "No AprilTag detected");
//        }
//
//        if (colorBlocks.size() > 0) {
//            telemetry.addData("Detected Color Blocks", colorBlocks.size());
//            for (ColorBlock block : colorBlocks) {
//                telemetry.addData("Block " + block.position,
//                        block.color + " at X:" + block.xCenter);
//            }
//        } else {
//            telemetry.addData("Color Blocks", "No color blocks detected");
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
//        return buffer != null &&
//                buffer.length >= 2 &&
//                buffer[OFFSET_FRAME_HEADER] == (byte) 0x55 &&
//                buffer[OFFSET_FRAME_HEADER + 1] == (byte) 0xAA;
//    }
//
//    // 内部类用于存储颜色区块信息
//    class ColorBlock {
//        String color;
//        int xCenter;
//        int yCenter;
//        int width;
//        int height;
//        int position; // 位置编号 (1, 2, 3)
//
//        public ColorBlock(String color, int xCenter, int yCenter, int width, int height, int position) {
//            this.color = color;
//            this.xCenter = xCenter;
//            this.yCenter = yCenter;
//            this.width = width;
//            this.height = height;
//            this.position = position;
//        }
//    }
//}
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
