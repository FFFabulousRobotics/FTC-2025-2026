import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MotorPIDController;

@TeleOp(name = "Elevation test")
public class ElevateTest extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        // 初始化电机
        DcMotor left = hardwareMap.get(DcMotor.class, "LL");
        DcMotor right = hardwareMap.get(DcMotor.class, "LR");
        waitForStart();
        float lift=0;
        MotorPIDController left_pid=new MotorPIDController(left,0.1,0.00005,0.0005);
        MotorPIDController right_pid=new MotorPIDController(right,0.1,0.00005,0.0005);
//        while(opModeIsActive())
//        {
//            if(gamepad1.right_trigger>0)
//            {
//                left_pid.setTarget(5750);
//                right_pid.setTarget(5922);
//            }
//            if(gamepad1.left_trigger>0)
//            {
//                left_pid.setTarget(10);
//                right_pid.setTarget(10);
//            }
//            left_pid.update();
//            right_pid.update();
            left.setPower(-1);
            right.setPower(-1);
            while(opModeIsActive());
//        }
//        left.setPower(0);
//        right.setPower(0);
    }
}