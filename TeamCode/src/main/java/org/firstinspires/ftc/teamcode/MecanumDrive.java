package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * 麦克纳姆轮驱动类
 * 支持平移和旋转的组合控制，带有轮子校准功能
 */
public class MecanumDrive {
    private final DcMotor leftFront, rightFront, leftRear, rightRear;
    private final double[] powerFactors;

    private final GoBildaPinpointDriver pp;
    /**
     * 构造函数
     * @param leftFront 左前电机
     * @param rightFront 右前电机
     * @param leftRear 左后电机
     * @param rightRear 右后电机
     * @param powerFactors 功率因数数组 [左前, 右前, 左后, 右后]
     */
    public MecanumDrive(DcMotor leftFront, DcMotor rightFront,
                        DcMotor leftRear, DcMotor rightRear,
                        GoBildaPinpointDriver pp,double[] powerFactors) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;

        // 复制功率因数
        this.powerFactors = new double[4];
        System.arraycopy(powerFactors, 0, this.powerFactors, 0, 4);
        this.pp=pp;
        pp.resetPosAndIMU();
        pp.setEncoderResolution(52, DistanceUnit.MM);
        pp.setOffsets(-30,15,DistanceUnit.MM);
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // 设置电机方向（根据实际安装调整）
        // 如果某些电机方向相反，可以在这里调整
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }
    public GoBildaPinpointDriver getIMU(){
        return pp;
    }
    public void drive(double heading, double power, boolean absolute, double rotation) {
        // 限制输入范围
        power = Math.max(-1, Math.min(1, power));
        rotation = Math.max(-1, Math.min(1, rotation));

        // 转换为弧度
        double headingRad;
        if(absolute)
            headingRad=Math.toRadians(pp.getHeading(AngleUnit.DEGREES)-heading);
        else
            headingRad = Math.toRadians(-heading);
        // 计算机器人坐标系下的运动分量
        double forward = power * Math.cos(headingRad);  // 前后分量
        double strafe = power * Math.sin(headingRad);   // 左右分量

        // 计算各轮功率（标准麦克纳姆轮公式）
        double lfPower = forward + strafe + rotation;
        double rfPower = forward - strafe - rotation;
        double lrPower = forward - strafe + rotation;
        double rrPower = forward + strafe - rotation;

        // 应用功率因数校准
        lfPower *= powerFactors[0];
        rfPower *= powerFactors[1];
        lrPower *= powerFactors[2];
        rrPower *= powerFactors[3];

        // 归一化功率，确保不超过最大值
        normalizePowers(new double[]{lfPower, rfPower, lrPower, rrPower});

        // 设置电机功率
        leftFront.setPower(lfPower);
        rightFront.setPower(rfPower);
        leftRear.setPower(lrPower);
        rightRear.setPower(rrPower);
    }

    /**
     * 归一化功率数组，确保所有值在[-1, 1]范围内
     * @param powers 功率数组
     */
    private void normalizePowers(double[] powers) {
        double maxPower = 0;
        for (double power : powers) {
            maxPower = Math.max(maxPower, Math.abs(power));
        }

        if (maxPower > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxPower;
            }
        }
    }

    /**
     * 设置单个电机的功率因数
     * @param wheelIndex 轮子索引 (0=左前, 1=右前, 2=左后, 3=右后)
     * @param factor 功率因数
     */
    public void setPowerFactor(int wheelIndex, double factor) {
        if (wheelIndex >= 0 && wheelIndex < 4) {
            powerFactors[wheelIndex] = factor;
        }
    }

    /**
     * 获取单个电机的功率因数
     * @param wheelIndex 轮子索引 (0=左前, 1=右前, 2=左后, 3=右后)
     * @return 功率因数
     */
    public double getPowerFactor(int wheelIndex) {
        if (wheelIndex >= 0 && wheelIndex < 4) {
            return powerFactors[wheelIndex];
        }
        return 1.0;
    }

    /**
     * 设置所有电机的功率因数
     * @param factors 功率因数数组 [左前, 右前, 左后, 右后]
     */
    public void setAllPowerFactors(double[] factors) {
        if (factors.length >= 4) {
            System.arraycopy(factors, 0, powerFactors, 0, 4);
        }
    }

    /**
     * 停止所有电机
     */
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * 设置电机运行模式
     * @param mode 电机模式
     */
    public void setMotorMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftRear.setMode(mode);
        rightRear.setMode(mode);
    }

    /**
     * 设置电机零功率行为
     * @param zeroPowerBehavior 零功率行为
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftRear.setZeroPowerBehavior(zeroPowerBehavior);
        rightRear.setZeroPowerBehavior(zeroPowerBehavior);
    }
}