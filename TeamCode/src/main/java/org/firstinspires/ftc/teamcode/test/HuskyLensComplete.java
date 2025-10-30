package org.firstinspires.ftc.teamcode.test;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.util.TypeConversion;

/**
 * 完整的HuskyLens FTC实现 - 基于DFRobot I2C协议V1.0
 * 修正了I2C写入方法签名问题
 */
@TeleOp(name = "HuskyLens Color Recognition", group = "Sensor")
public class HuskyLensComplete extends LinearOpMode {

    private HuskyLensI2C huskyLens;
    private float myFloatVariable;

    @Override
    public void runOpMode() {
        initHuskyLens();

        telemetry.addData("状态", "初始化完成，等待开始");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 设置算法为颜色识别
            huskyLens.writeAlgorithm(HuskyLensI2C.ALGORITHM_COLOR_RECOGNITION);

            // 请求数据
            huskyLens.request();

            // 读取块参数
            HuskyLensI2C.BlockParameter block = huskyLens.readBlockParameterDirect(1);
            if (block != null) {
                myFloatVariable = block.ID;
                telemetry.addData("块参数值", "%.1f", myFloatVariable);
                telemetry.addData("块位置", "X: %d, Y: %d", block.x, block.y);
                telemetry.addData("块尺寸", "宽: %d, 高: %d", block.width, block.height);
            } else {
                telemetry.addData("状态", "未检测到块");
            }

            // 检测错误
            if (huskyLens.detectError()) {
                telemetry.addData("警告", "检测到通信错误");
            }

            telemetry.update();
            sleep(100);
        }
    }

    private void initHuskyLens() {
        try {
            I2cDeviceSynch i2cDevice = hardwareMap.get(I2cDeviceSynch.class, "huskylens");
            huskyLens = new HuskyLensI2C(i2cDevice);
            telemetry.addData("HuskyLens", "初始化成功");
        } catch (Exception e) {
            telemetry.addData("HuskyLens错误", "初始化失败: " + e.getMessage());
        }
    }

    /**
     * HuskyLens I2C设备实现 - 修正了write方法签名问题
     */
    public static class HuskyLensI2C extends I2cDeviceSynchDevice<I2cDeviceSynch> {

        private static final int I2C_ADDRESS = 0x32;
        private static final int PID_HAND = 0x7E;
        private static final int PID_ERROR = 0x7F;

        // 算法类型
        public static final int ALGORITHM_COLOR_RECOGNITION = 0x03;
        public static final int ALGORITHM_FACE_RECOGNITION = 0x00;
        public static final int ALGORITHM_OBJECT_TRACKING = 0x01;
        public static final int ALGORITHM_LINE_TRACKING = 0x02;
        public static final int ALGORITHM_TAG_RECOGNITION = 0x04;
        public static final int ALGORITHM_OBJECT_CLASSIFICATION = 0x05;

        // 寄存器映射
        private static final int REG_COMMAND = 0x00;
        private static final int REG_REQUEST = 0x20;

        private byte[] errorBuffer = new byte[5];

        public HuskyLensI2C(I2cDeviceSynch deviceClient) {
            super(deviceClient, true);
            this.deviceClient.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));
            super.registerArmingStateCallback(false);
            this.deviceClient.engage();
        }

        @Override
        protected boolean doInitialize() {
            return true;
        }

        @Override
        public Manufacturer getManufacturer() {
            return Manufacturer.Other;
        }

        @Override
        public String getDeviceName() {
            return "DFRobot HuskyLens";
        }

        /**
         * 生成PID（带奇校验位）
         */
        private byte generatePID(byte pid) {
            int count = Integer.bitCount(pid & 0xFF);
            if (count % 2 == 1) {
                return (byte) (pid << 1);
            } else {
                return (byte) ((pid << 1) | 1);
            }
        }

        /**
         * 设置算法 - 修正了write方法调用
         */
        public boolean writeAlgorithm(int algorithm) {
            byte[] command = new byte[6];
            command[0] = generatePID((byte) REG_COMMAND);
            command[1] = (byte) 0x00;
            command[2] = (byte) 0x01;
            command[3] = (byte) algorithm;
            command[4] = (byte) (command[1] + command[2] + command[3]);
            command[5] = generatePID((byte) PID_HAND);

            // 修正：使用正确的write方法签名
            deviceClient.write(0x50, command);
            return true;
        }

        /**
         * 请求数据 - 修正了write方法调用
         */
        public boolean request() {
            byte[] requestCommand = new byte[5];
            requestCommand[0] = (byte) 0x55;
            requestCommand[1] = (byte) 0xAA;
            requestCommand[2] = (byte) 0x20;
            requestCommand[3] = (byte) 0x01;
            requestCommand[4] = (byte) 0x00;

            // 修正：使用正确的write方法签名
            deviceClient.write(0x50, requestCommand);
            return true;
        }

        /**
         * 读取块参数
         */
        public BlockParameter readBlockParameterDirect(int blockId) {
            if (!request()) {
                return null;
            }

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return null;
            }

            byte[] readBuffer = deviceClient.read(0x51, 32);
            if (readBuffer == null || readBuffer.length < 15) {
                return null;
            }

            // 检查帧头
            if (readBuffer[0] != 0x55 || readBuffer[1] != 0xAA || readBuffer[2] != 0x20) {
                return null;
            }

            int dataLength = (readBuffer[3] & 0xFF) | ((readBuffer[4] & 0xFF) << 8);
            if (dataLength < 7) {
                return null;
            }

            BlockParameter block = new BlockParameter();
            int offset = 5;

            block.ID = readBuffer[offset] & 0xFF;
            block.x = TypeConversion.byteArrayToShort(new byte[]{readBuffer[offset + 1], readBuffer[offset + 2]});
            block.y = TypeConversion.byteArrayToShort(new byte[]{readBuffer[offset + 3], readBuffer[offset + 4]});
            block.width = TypeConversion.byteArrayToShort(new byte[]{readBuffer[offset + 5], readBuffer[offset + 6]});
            block.height = TypeConversion.byteArrayToShort(new byte[]{readBuffer[offset + 7], readBuffer[offset + 8]});

            return block;
        }

        /**
         * 读取所有检测到的块
         */
        public Blocks readBlocks() {
            if (!request()) {
                return new Blocks();
            }

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return new Blocks();
            }

            byte[] data = deviceClient.read(0x51, 64);
            if (data == null || data.length < 5) {
                return new Blocks();
            }

            if (data[0] != 0x55 || data[1] != 0xAA) {
                return new Blocks();
            }

            Blocks blocks = new Blocks();
            int dataLength = (data[3] & 0xFF) | ((data[4] & 0xFF) << 8);

            if (dataLength >= 6) {
                int blockCount = data[5] & 0xFF;

                for (int i = 0; i < blockCount && i < 10; i++) {
                    int offset = 6 + i * 7;
                    if (offset + 6 >= data.length) break;

                    BlockParameter block = new BlockParameter();
                    block.ID = data[offset] & 0xFF;
                    block.x = TypeConversion.byteArrayToShort(new byte[]{data[offset + 1], data[offset + 2]});
                    block.y = TypeConversion.byteArrayToShort(new byte[]{data[offset + 3], data[offset + 4]});
                    block.width = TypeConversion.byteArrayToShort(new byte[]{data[offset + 5], data[offset + 6]});
                    block.height = block.width;

                    blocks.addBlock(block);
                }
            }

            return blocks;
        }

        /**
         * 检测错误
         */
        public boolean detectError() {
            byte[] errorData = deviceClient.read(0x52, 5);
            if (errorData == null || errorData.length < 5) {
                return false;
            }

            if (errorData[0] == generatePID((byte) PID_ERROR)) {
                if (errorData[1] != 0) {
                    byte checksum = (byte) (errorData[1] + errorData[2] + errorData[3]);
                    return errorData[4] == checksum;
                }
            }
            return false;
        }

        /**
         * 简化版写入方法 - 用于寄存器操作
         */
        private boolean writeData(int register, byte[] data) {
            // 修正：使用正确的write方法签名
            deviceClient.write(register, data);
            return true;
        }

        /**
         * 块参数类
         */
        public static class BlockParameter {
            public int ID;
            public int x;
            public int y;
            public int width;
            public int height;

            @Override
            public String toString() {
                return String.format("Block{ID=%d, x=%d, y=%d, width=%d, height=%d}",
                        ID, x, y, width, height);
            }
        }

        /**
         * 块集合类
         */
        public static class Blocks {
            private BlockParameter[] blocks = new BlockParameter[10];
            private int count = 0;

            public void addBlock(BlockParameter block) {
                if (count < 10) {
                    blocks[count++] = block;
                }
            }

            public BlockParameter getBlock(int index) {
                if (index >= 0 && index < count) {
                    return blocks[index];
                }
                return null;
            }

            public int count() {
                return count;
            }

            public boolean available() {
                return count > 0;
            }
        }
    }
}