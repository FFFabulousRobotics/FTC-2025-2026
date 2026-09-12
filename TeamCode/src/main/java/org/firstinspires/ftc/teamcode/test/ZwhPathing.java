package org.firstinspires.ftc.teamcode.test;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GenericPIDController;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class ZwhPathing {

    private final MecanumDrive md;
    private final double offset;

    // PID
    private final GenericPIDController distancePID;
    private final GenericPIDController anglePID;

    // target
    private double targetX;
    private double targetY;
    private double targetHeading;

    // robot state
    private double X;
    private double Y;
    private double Heading;

    // config
    private double maxPower;
    private final double rotationTolerance;
    private final double distanceTolerance;

    // PID params cache
    private double[] dParams;
    private double[] aParams;

    // telemetry
    public double dx, dy;
    public double moveHeading;
    public double movePower;
    public double rotationPower;
    public double distance;
    public double angleError;

    // state
    private boolean nearTarget = false;

    // heading freeze
    private double lastMoveHeading = 0;

    // timing
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    public ZwhPathing(
            MecanumDrive drive,
            double offsetDegrees,
            double[] distancePIDParams,
            double[] anglePIDParams,
            double maxPower,
            double rotationTolerance,
            double distanceTolerance
    ) {

        this.md = drive;

        drive.getIMU().resetPosAndIMU();
        sleep(1000);

        this.offset = offsetDegrees;

        dParams = distancePIDParams;
        aParams = anglePIDParams;

        distancePID = new GenericPIDController(
                dParams[0],
                dParams[1],
                dParams[2]
        );

        anglePID = new GenericPIDController(
                aParams[0],
                aParams[1],
                aParams[2]
        );

        distancePID.setTarget(0);
        anglePID.setTarget(0);

        this.maxPower = maxPower;
        this.rotationTolerance = rotationTolerance;
        this.distanceTolerance = distanceTolerance;

        updateRobotState();
    }

    // =========================
    // Target
    // =========================

    public void setTarget(double x, double y) {

        targetX = x;
        targetY = y;

        targetHeading =
                md.getIMU().getHeading(AngleUnit.DEGREES);

        resetControllers();
    }

    public void setTarget(
            double x,
            double y,
            double headingDegrees
    ) {

        targetX = x;
        targetY = y;
        targetHeading = headingDegrees;

        resetControllers();
    }

    // =========================
    // PID Params
    // =========================

    public void setDistancePIDParams(
            double kp,
            double ki,
            double kd
    ) {

        dParams = new double[]{kp, ki, kd};

        distancePID.setPidCoefficients(
                kp,
                ki,
                kd
        );
    }

    public void setAnglePIDParams(
            double kp,
            double ki,
            double kd
    ) {

        aParams = new double[]{kp, ki, kd};

        anglePID.setPidCoefficients(
                kp,
                ki,
                kd
        );
    }

    // =========================
    // Reset
    // =========================

    private void resetControllers() {

        distancePID.reset();
        anglePID.reset();

        nearTarget = false;

        updateRobotState();

        dx = targetX - X;
        dy = targetY - Y;

        lastMoveHeading =
                toDegrees(atan2(dy, dx));

        timer.reset();
        lastTime = timer.seconds();
    }

    // =========================
    // Robot State
    // =========================

    private void updateRobotState() {

        md.getIMU().update();

        X = md.getIMU().getPosX(DistanceUnit.MM);

        Y = md.getIMU().getPosY(DistanceUnit.MM);

        Heading =
                md.getIMU().getHeading(
                        AngleUnit.DEGREES
                ) - offset;
    }

    // =========================
    // Utils
    // =========================

    private double normalizeAngle180(double angle) {

        angle %= 360;

        if(angle > 180)
            angle -= 360;

        if(angle < -180)
            angle += 360;

        return angle;
    }

    private double clip(
            double val,
            double min,
            double max
    ) {

        return Math.max(
                min,
                Math.min(max, val)
        );
    }

    // =========================
    // Public State
    // =========================

    public double getDistanceToTarget() {
        return distance;
    }

    public double getHeadingError() {
        return angleError;
    }

    public boolean isAtTarget() {

        return
                abs(distance) < distanceTolerance
                        &&
                        abs(angleError) < rotationTolerance;
    }

    public void setMaxPower(double mp) {
        maxPower = mp;
    }

    // =========================
    // Main Update
    // =========================

    public boolean update() {

        updateRobotState();

        // =========================
        // timing
        // =========================

        double now = timer.seconds();
        double dt = now - lastTime;

        lastTime = now;

        // avoid divide issues
        if(dt <= 0)
            dt = 0.001;

        // =========================
        // errors
        // =========================

        dx = targetX - X;
        dy = targetY - Y;

        distance = hypot(dx, dy);

        angleError =
                normalizeAngle180(
                        targetHeading - Heading
                );

        // =========================
        // movement heading
        // =========================

        // freeze heading near target
        if(distance > 60) {

            moveHeading =
                    toDegrees(atan2(dy, dx));

            lastMoveHeading = moveHeading;

        }
        else {

            moveHeading = lastMoveHeading;
        }

        // =========================
        // PID stage switching
        // =========================

        if(!nearTarget &&
                distance < 120) {

            nearTarget = true;

            // softer PID near target
            distancePID.setPidCoefficients(
                    dParams[0] * 0.5,
                    dParams[1],
                    dParams[2] * 0.2
            );
        }

        // =========================
        // translation
        // =========================

        if(distance > distanceTolerance) {

            movePower =
                    -distancePID.update(distance);

            // distance slow down
            if(distance < 100) {

                movePower *=
                        distance / 100.0;
            }

            // minimum overcome friction
            if(abs(movePower) < 0.08) {

                movePower =
                        0.08 *
                                Math.signum(movePower);
            }

            movePower = clip(
                    movePower,
                    -maxPower,
                    maxPower
            );

        }
        else {

            movePower = 0;
        }

        // =========================
        // rotation
        // =========================

        if(abs(angleError) > rotationTolerance) {

            rotationPower =
                    anglePID.update(angleError);

            // soften near target
            if(abs(angleError) < 10) {

                rotationPower *= 0.5;
            }

            rotationPower = clip(
                    rotationPower,
                    -0.4,
                    0.4
            );

        }
        else {

            rotationPower = 0;
        }

        // =========================
        // final drive
        // =========================

        md.drive(
                moveHeading,
                movePower,
                true,
                rotationPower
        );

        // =========================
        // finished
        // =========================

        if(
                abs(distance) < distanceTolerance
                        &&
                        abs(angleError) < rotationTolerance
        ) {

            md.drive(0,0,true,0);

            return true;
        }

        return false;
    }
}
