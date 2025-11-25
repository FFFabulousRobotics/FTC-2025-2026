import static java.lang.Math.floor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MotorPIDController;

@TeleOp(name = "Elevation test")
public class ElevateTest extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        // 初始化电机
        DcMotor left = hardwareMap.get(DcMotor.class, "FL");
        DcMotor right = hardwareMap.get(DcMotor.class, "FR");
        waitForStart();
        float lift=0;
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorPIDController left_pid=new MotorPIDController(left,0.1,0.0005,-0.005);
        MotorPIDController right_pid=new MotorPIDController(right,0.1,0.0005,-0.005);
        while(opModeIsActive())
        {
            if(gamepad1.right_trigger>0)
            {
                left_pid.setTarget(-5750);
                right_pid.setTarget(-5922);
            }
            if(gamepad1.left_trigger>0)
            {
                left_pid.setTarget(-10);
                right_pid.setTarget(-10);
            }
            left_pid.update();
            right_pid.update();
        }
    }
}