package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * Take control of all the robot hardware.
 *
 * @author @CZ-Tech @LeChocolatChaud
 */
@SuppressWarnings({"unused"})
public class RobotHardwareImpl implements RobotHardware {

    // *****
    // members
    // *****

    // super providers
    private final LinearOpMode opMode;      // the calling OpMode
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // computer vision
    private VisionPortal visionPortal;

    public AprilTagProcessor aprilTag;

    //public TfodProcessor tfod;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/NewCubes19726B3.tflite";
    private static final String[] LABELS = {
            "BlueCube",
            "RedCube"
    };

    // hardware handles
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor armMotor = null;
    private DcMotor liftLeftMotor = null;
    private DcMotor liftRightMotor = null;
    private DcMotor intakeMotor = null;
    private Servo dumpServo = null;
    private Servo droneServo = null;
    private Servo holderServo = null;
    private Rev2mDistanceSensor distanceSensor = null;

    private IMU imu = null;
    private SparkFunOTOS otos = null;


    // auto drive
    private double headingError = 0;
    @SuppressWarnings("FieldCanBeLocal")
    private double targetHeading = 0;
    @SuppressWarnings("FieldMayBeFinal")
    private double driveSpeed = 0;
    private double turnSpeed = 0;

    // *****
    // constants
    // *****
    // TODO: Change those constants to suit the actual needs

    private static final double HAND_SPEED = 0.02;  // sets rate to move servo

    // counts per inch (for precise movement)

    // standard: 28
    // REV (40:1): 28*40=1120
    // REV (20:1): 28*20=560
    // *unit: counts/revolution
    static final double COUNTS_PER_MOTOR_REV = 560;

    // gearing UP (will affect the direction of wheel rotation) < 1 < gearing DOWN
    // eg. for a 12-tooth driving a 24-tooth, the value is 24/12=2.0
    static final double DRIVE_GEAR_REDUCTION = 1.0;

    // wheel diameter in inches
    static final double WHEEL_DIAMETER_INCHES = 5.31;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // desired driving/control characteristics
    static final double HEADING_THRESHOLD = 0.5;    // how close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    static final double P_STRAFE_GAIN = 0.03;   // Strafe Speed Control "Gain".
    static final double P_TURN_GAIN = 0.1;     // Larger is more responsive, but also less stable

    final double X_OFFSET = 5.9;
    final double Y_OFFSET = 2.36;

    private static final double AXIAL_REDUCTION = 0.5;
    private static final double LATERAL_REDUCTION = 0.5;
    private static final double YAW_REDUCTION = 0.33;

    public RobotHardwareImpl(LinearOpMode opMode) {
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
    }


    // ########################################################################################
    // !!!                                  BASIC MOVEMENTS                                 !!!
    // ########################################################################################

    @Override
    public void initMovement() {
        // init motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeftMotor = hardwareMap.get(DcMotor.class, "LiftLeft");
        liftRightMotor = hardwareMap.get(DcMotor.class, "LiftRight");
        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        liftLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftRightMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        dumpServo = hardwareMap.get(Servo.class, "Dump");
        droneServo = hardwareMap.get(Servo.class, "Drone");
        holderServo = hardwareMap.get(Servo.class, "Holder");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "Distance");

        // hub orientation
        // TODO: EDIT these two lines to match the actual mounting configuration
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        otos = opMode.hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();
        telemetry.addData(">", "Hardware Initialized");
//        telemetry.addData("OMGGG", hardwareConfig.hardware.chassis.motors.getClass().getName());
        telemetry.update();
    }

    /**
     * Move based on field coords.
     *
     * @param axial   Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral Right/Left driving power (-1.0 to 1.0) +ve is forward
     * @param yaw     Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    @Override
    public void driveRobotFieldCentric(double axial, double lateral, double yaw) {
        double botHeading = getHeading(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
        driveRobot(rotY, rotX, yaw);
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial   Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral Right/Left driving power (-1.0 to 1.0) +ve is forward
     * @param yaw     Right/Left turning power (-1.0 to 1.0) +ve is clockwise
     */
    @Override
    public void driveRobot(double axial, double lateral, double yaw) {
        telemetry.addData("Speed", "Vy %5.2f, Vx %5.2f, Vr %5.2f ", axial, lateral, yaw);
        telemetry.addData("pos", "x %5.2f, y %5.2f, h %5.2f", getPosition().x, getPosition().y, getPosition().h);
        axial *= AXIAL_REDUCTION;
        lateral *= LATERAL_REDUCTION;
        yaw *= YAW_REDUCTION;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double leftFrontPower = axial - lateral - yaw;
        double rightFrontPower = axial + lateral + yaw;
        double leftBackPower = axial + lateral - yaw;
        double rightBackPower = axial - lateral + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /**
     * Set the power of every motor.
     *
     * @param leftFrontPower  Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontPower Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackPower   Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackPower  Forward/Rev driving power (-1.0 to 1.0) +ve is forward
     * @return The {@link RobotHardwareImpl} instance.
     */
    @Override
    public RobotHardware setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        telemetry.addData("Front Left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  Left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        return this;
    }

    /**
     * Stop All drive motors.
     */
    @Override
    public void stopMotor() {
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Set drive wheels run mode for all drive motors.
     *
     * @param mode The desired run mode.
     */
    @Override
    public void setRunMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    /**
     * Set zero power behavior for all drive motors.
     *
     * @param behavior The desired zero power behaviour.
     */
    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * Set target position for all drive motors for moving forward.
     *
     * @param moveCounts The desired increment/decrement.
     */
    @Override
    public void setTargetPosition(int moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + moveCounts);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + moveCounts);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + moveCounts);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + moveCounts);
    }

    /**
     * Set target position for all drive motors for moving sideways.
     *
     * @param moveCounts The desired increment/decrement.
     */
    @Override
    public void setStrafeTargetPosition(int moveCounts) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + moveCounts);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - moveCounts);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - moveCounts);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + moveCounts);
    }

    /**
     * Check if all drive motors are busy (not yet at target position).
     *
     * @return Drives busy state.
     */
    @Override
    public boolean isAllBusy() {
        return leftFrontDrive.isBusy() &&
                leftBackDrive.isBusy() &&
                rightFrontDrive.isBusy() &&
                rightBackDrive.isBusy();
    }

    /**
     * Set the arm motor power.
     * Support link calls.
     *
     * @param power Arm motor power (-1.0 to 1.0).
     * @return The RobotHardware instance.
     */
    @Override
    public RobotHardware setArmPower(double power) {
        armMotor.setPower(power);
        return this;
    }

    /**
     * Set the lift motor power.
     * Support link calls.
     *
     * @param power Lift motor power (-1.0 to 1.0).
     * @return The RobotHardware instance.
     */
    @Override
    public RobotHardware setLiftPower(double power) {
        if (power < 0) {
            liftLeftMotor.setPower(power);
            liftRightMotor.setPower(power);
        } else {
            liftLeftMotor.setPower(power);
            liftRightMotor.setPower(power);
        }
        return this;
    }

    @Override
    public int getLiftPosition() {
        return liftLeftMotor.getCurrentPosition();
    }

    /**
     * Set the intake motor power.
     * Support link calls.
     *
     * @param power n motor power (-1.0 to 1.0).
     * @return The RobotHardware instance.
     */
    @Override
    public RobotHardware setIntakePower(double power) {
        intakeMotor.setPower(power);
        return this;
    }

    @Override
    public RobotHardware setDumpPosition(double position) {
        dumpServo.setPosition(position);
        return this;
    }

    @Override
    public double getDumpPosition() {
        return dumpServo.getPosition();
    }

    @Override
    public RobotHardware setDronePosition(double position) {
        droneServo.setPosition(position);
        return this;
    }

    @Override
    public RobotHardware setHolderPosition(double position) {
        holderServo.setPosition(position);
        return this;
    }


    // ########################################################################################
    // !!!                               COMPUTER VISION                                   !!!
    // ########################################################################################


    /**
     * Init computer vision.
     */
    @Override
    public void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(669.644585545, 669.644585545, 626.351066943, 312.353620695)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);


        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

//        tfod = new TfodProcessor.Builder()
//                // With the following lines commented out, the default TfodProcessor Builder
//                // will load the default model for the season. To define a custom model to load,
//                // choose one of the following:
//                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
//                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                //.setModelAssetName(TFOD_MODEL_ASSET)
//                .setModelFileName(TFOD_MODEL_FILE)
//                // The following default settings are available to un-comment and edit as needed to
//                // set parameters for custom models.
//                .setModelLabels(LABELS)
//                .setIsModelTensorFlow2(true)
//                //.setIsModelQuantized(true)
//                .setModelInputSize(300)
//                .setModelAspectRatio(640.0 / 480.0)
//                .build();
//
//        tfod.setMinResultConfidence(0.5f);

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                //.addProcessors(tfod, aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();
        visionPortal.getCameraState();
    }

    /**
     * Close the computer vision handle.
     */
    @Override
    public void closeVision() {
        visionPortal.close();
    }

    /**
     * Get the AprilTag detection with the specified ID.
     *
     * @param desiredTagId The desired tag id.
     * @return The matching {@link AprilTagDetection} object, or null if no match is found.
     */
    @Override
    public AprilTagDetection getAprilTag(int desiredTagId) {

        boolean targetFound = false;
        AprilTagDetection desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }

        return desiredTag;
    }

    /**
     * Move the robot to the desired tag.
     *
     * @param desiredTag      The tag detection object.
     * @param desiredDistance The desired distance from the tag.
     */
    @Override
    public void moveToAprilTag(AprilTagDetection desiredTag, double desiredDistance) {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double range = desiredTag.ftcPose.range;
        double bearing = desiredTag.ftcPose.bearing;
        double yaw = desiredTag.ftcPose.yaw;

        double xNew = bearing > 0 ? range * Math.sin(bearing) + X_OFFSET : range * Math.sin(bearing) - X_OFFSET;
        double yNew = range * Math.cos(bearing) + Y_OFFSET;
        double rangeNew = Math.sqrt(Math.pow(xNew, 2) + Math.pow(yNew, 2));
        double bearingNew = Math.toDegrees(Math.atan2(xNew, yNew));
        double yawNew = Math.toDegrees(yaw);

        double rangeError = (rangeNew - desiredDistance);
        double headingError = bearingNew;
        double yawError = yawNew;

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive = rangeError * Math.cos(bearingNew);
        double strafe = rangeNew * Math.sin(bearingNew);
        double turn = -yawError;

        double heading = getHeading();

        driveStraight(0.5, drive, heading);
        driveStrafe(0.5, strafe, heading);
        turnToHeading(0.3, heading + turn);
    }

    @Override
    public void setTagDecimation(float decimation) {
        aprilTag.setDecimation(decimation);
    }
//
//    /**
//     * Get the object detection with the specified ID.
//     *
//     * @param desiredLabel The desired object label.
//     * @return The matching {@link Recognition} object, or null if no match is found.
//     */
    @Override
//    public Recognition getTfod(String desiredLabel) {
//        Recognition desiredTfod = null;
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            //            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            //            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//            double x = recognition.getLeft();
//            double y = recognition.getTop();
//            telemetry.addData("", " ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//
//            if (desiredLabel.equals(recognition.getLabel())) {
//                desiredTfod = recognition;
//                break;
//            }
//        }   // end for() loop
//
//        return desiredTfod;
//    }
//
//    /**
//     * Move the robot to the desired object.
//     *
//     * @param desiredTfod          The recognition object.
//     * @param desiredWidthInScreen The desired width of the object as seen in the camera.
//     */
//    @Override
//    public void moveToTfod(Recognition desiredTfod, double desiredWidthInScreen) {
//        double x = (desiredTfod.getLeft() + desiredTfod.getRight()) / 2;
//        double y = (desiredTfod.getTop() + desiredTfod.getBottom()) / 2;
//        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//        double rangeError = (1 - desiredTfod.getWidth() / desiredWidthInScreen);
//        //        double  yawError        = desiredTag.ftcPose.yaw;
//        double headingError = 320 - x;
//        driveRobot(
//                rangeError * P_DRIVE_GAIN,
//                0,
//                headingError * P_TURN_GAIN
//        );
//    }
    // ########################################################################################
    // !!!                                    GYROSCOPE                                     !!!
    // ########################################################################################

    /**
     * Read the robot heading directly from the IMU.
     *
     * @return The heading of the robot in degrees.
     */
    //@Override
    public double getHeading() {
        return getHeading(AngleUnit.DEGREES);
    }

    /**
     * read the Robot heading directly from the IMU
     *
     * @param unit The desired angle unit (degrees or radians)
     * @return The heading of the robot in desired units.
     */
    @Override
    public double getHeading(AngleUnit unit) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("Yaw/Pitch/Roll", orientation.toString());
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        return orientation.getYaw(unit);
    }

    /**
     * Reset imu yaw.
     */
    @Override
    public void resetYaw() {
        imu.resetYaw();
    }

    // ########################################################################################
    // !!!                                AUTONOMOUS PERIOD                                 !!!
    // ########################################################################################

    @Override
    public RobotHardware forward(double d) {
        return driveStraight(0.55, d, getHeading());
    }

    @Override
    public RobotHardware fastForward(double d, int step) {
        for (int i = 0; i < step; i++) {
            forward(d / step);
        }
        return this;
    }

    @Override
    public RobotHardware fastForward(double d) {
        return driveStraight(0.7, d, getHeading());
    }

    @Override
    public RobotHardware backward(double d) {
        return driveStraight(0.6, -d, getHeading());
    }

    @Override
    public RobotHardware fastBackward(double d, int step) {
        for (int i = 0; i < step; i++) {
            fastBackward(d / step);
        }
        return this;
    }


    @Override
    public RobotHardware fastBackward(double d) {
        return driveStraight(0.7, -d, getHeading());
    }

    @Override
    public RobotHardware rightShift(double d) {
        return driveStrafe(0.7, d, getHeading());
    }

    @Override
    public RobotHardware leftShift(double d) {
        return driveStrafe(0.7, -d, getHeading());
    }

    @Override
    public RobotHardware spin(double h) {
        return turnToHeading(0.6, h);
    }

    @Override
    public RobotHardware fastSpin(double h) {
        return turnToHeading(0.7, h);
    }

    @Override
    public RobotHardware sleep(long milliseconds) {
        opMode.sleep(milliseconds);
        return this;
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     * Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = forward. +ve is counter-clockwise from forward. -ve is clockwise from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */

    @Override
    public RobotHardware driveStraight(double maxDriveSpeed,
                                       double distance,
                                       double heading) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            driveRobot(maxDriveSpeed, 0, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                driveRobot(maxDriveSpeed, 0, -turnSpeed);
                //                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    @Override
    @Deprecated
    public RobotHardware driveStrafe(double distance) {
        return driveStrafe(0.5, distance, 0);
    }

    @Override
    public RobotHardware driveStrafe(double maxDriveSpeed,
                                     double distance,
                                     double heading
    ) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            setStrafeTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            driveRobot(0, maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                driveRobot(0, maxDriveSpeed, -turnSpeed);
                //                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    // ########################################################################################
    // !!!                                    WIP                                           !!!
    // ########################################################################################

    @Override
    public void turnHead() {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    /**
     * Spin on the central axis to point in a new direction.
     * <p>
     * Move will stop if either of these conditions occur:
     * <p>
     * 1) Move gets to the heading (angle)
     * <p>
     * 2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = forward. +ve is counter-clockwise from forward. -ve is clockwise from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     */
    @Override
    public RobotHardware turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        //绝对值的问题？
        while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            driveRobot(0, 0, -turnSpeed);
            //            telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f",maxTurnSpeed,turnSpeed,heading,getHeading());
            telemetry.update();
        }

        // Stop all motion;
        stopMotor();
        return this;
    }

    /**
     * Obtain & hold a heading for a finite amount of time
     * <p>
     * Move will stop once the requested time has elapsed
     * <p>
     * This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed Maximum differential turn speed (range 0 to +1.0)
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                     0 = forward. +ve is counter-clockwise from forward. -ve is clockwise from forward.
     *                     If a relative angle is required, add/subtract from current heading.
     * @param holdTime     Length of time (in seconds) to hold the specified heading.
     */
    @Override
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            driveRobot(0, 0, turnSpeed);

        }

        // Stop all motion;
        stopMotor();
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    @Override
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    @Override
    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    @Override
    public RobotHardware gotoDistance(double target_distance, double init_distance) {
        if (Math.abs(target_distance - getDistance() - init_distance) < 1) {
            fastForward(-(getDistance() - target_distance));
        } else {
            fastForward(-init_distance);
        }
        return this;
    }

    @Override
    public double[] SpinVector(double[] vector, double angle) {
        double x = vector[0] * Math.cos(Math.toRadians(angle)) - vector[1] * Math.sin(Math.toRadians(angle));
        double y = vector[0] * Math.sin(Math.toRadians(angle)) + vector[1] * Math.cos(Math.toRadians(angle));
        return new double[]{x, y};
    }

    @Override
    public double[] getDisplacement(double[] CurrentPos, double[] DesiredPos) {
        double CurrentX = CurrentPos[0];
        double CurrentY = CurrentPos[1];
        double CurrentHeading = CurrentPos[2];
        double DesiredX = DesiredPos[0];
        double DesiredY = DesiredPos[1];
        double DesiredHeading = DesiredPos[2];
        double[] Displacement = {DesiredX - CurrentX, DesiredY - CurrentY, CurrentHeading};
        Displacement = SpinVector(Displacement, -CurrentHeading);
        return Displacement;
    }

    /**
     * Go to the position given (track only include left/right and forward/backward)
     * Go forward/backward first,then move left/right.
     *
     * @param CurrentPos The current position.(position{axial,lateral,heading})
     * @param DesiredPos The desired position.(position{axial,lateral,heading})
     * @return RobotHardware class.
     */
    @Override
    public RobotHardware gotoPosition(double[] CurrentPos, double[] DesiredPos) {
        double[] Displacement = getDisplacement(CurrentPos, DesiredPos);
        double DesiredHeading = DesiredPos[2];
        return fastForward(-Displacement[0])
                .leftShift(-Displacement[1])
                .fastSpin(DesiredHeading);
    }

    /**
     * Go to desired position(move forward/backward first,then left/right)
     */
    @Override
    public RobotHardware gotoPosition(double x, double y, double h) {
        SparkFunOTOS.Pose2D CurrentPos = getPosition();
        double[] DesiredPos = {x, y, h};
        return gotoPosition(new double[]{CurrentPos.x, CurrentPos.y, CurrentPos.h}, DesiredPos);
    }

    /**
     * Go to the position given (track only include left/right and forward/backward)
     * Move left/right first,then go forward/backward.
     *
     * @param CurrentPos The current position.(position{axial,lateral,heading})
     * @param DesiredPos The desired position.(position{axial,lateral,heading})
     * @return RobotHardware class.
     */
    @Override
    public RobotHardware gotoPosition2(double[] CurrentPos, double[] DesiredPos) {
        double[] Displacement = getDisplacement(CurrentPos, DesiredPos);
        double DesiredHeading = DesiredPos[2];
        return leftShift(-Displacement[1])
                .fastForward(-Displacement[0])
                .fastSpin(DesiredHeading);
    }

    /**
     * Go to desired position(Move left/right first,then forward/backward)
     */
    @Override
    public RobotHardware gotoPosition2(double x, double y, double h) {
        SparkFunOTOS.Pose2D CurrentPos = getPosition();
        double[] DesiredPos = {x, y, h};
        return gotoPosition2(new double[]{CurrentPos.x, CurrentPos.y, CurrentPos.h}, DesiredPos);
    }

    /**
     * Go to Desired position by moving diagonally and moving straight.
     *
     * @param CurrentPos Current position(position{x,y,heading})
     * @param DesiredPos Desired position(position{x,y,heading})
     * @return RobotHardware class
     */
    @Override
    public RobotHardware fastGotoPosition(double[] CurrentPos, double[] DesiredPos) {
        double[] Displacement = getDisplacement(CurrentPos, DesiredPos);
        double DesiredHeading = DesiredPos[2];
        double deltaX = Displacement[0];
        double deltaY = Displacement[1];

        int angle = 45;
        if (deltaX >= 0 && deltaY >= 0) {
            angle = 45;
        } else if (deltaX >= 0 && deltaY <= 0) {
            angle = -45;
        } else if (deltaX <= 0 && deltaY >= 0) {
            angle = 135;
        } else if (deltaX <= 0 && deltaY <= 0) {
            angle = -135;
        }
        double diagonalDistance = Math.min(Math.abs(deltaX), Math.abs(deltaY));
        return moveDiagonally(diagonalDistance, angle)
                .gotoPosition(DesiredPos[0], DesiredPos[1], DesiredPos[2]);
    }

    /**
     * Go to desired position(Move diagonally first,then straight)
     */
    @Override
    public RobotHardware fastGotoPosition(double x, double y, double h) {
        SparkFunOTOS.Pose2D CurrentPos = getPosition();
        double[] DesiredPos = {x, y, h};
        return fastGotoPosition(new double[]{CurrentPos.x, CurrentPos.y, CurrentPos.h}, DesiredPos);
    }

    @Override
    public void setDiagonalTargetPosition(int moveCounts, double angle) {
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + moveCounts);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition());
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + moveCounts);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition());
    }

    /**
     * Move diagonally in 45degrees.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance      The distance of moving 45degrees(The variation of x and y,
     *                      without multiplying sqrt2)
     * @param angle         The angle of moving diagonally.It must be +-45/+-145 degrees.
     *                      +angle is counter-clockwise from the robot's current heading.
     *                      -angle is clockwise from the robot's current heading.
     * @param heading       Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                      0 = forward. +ve is counter-clockwise from forward. -ve is clockwise from forward.
     *                      If a relative angle is required, add/subtract from the current robotHeading.
     */
    @Override
    public RobotHardware driveDiagonal(double maxDriveSpeed,
                                       double distance,
                                       int angle,
                                       double heading) {
        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            //Analyse maxDriveSpeed
            distance = -distance;
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            double maxDriveSpeedX;
            double maxDriveSpeedY;
            switch (angle) {
                case 45:
                    maxDriveSpeedX = maxDriveSpeed;
                    maxDriveSpeedY = maxDriveSpeed;
                    break;
                case -45:
                    maxDriveSpeedX = maxDriveSpeed;
                    maxDriveSpeedY = -maxDriveSpeed;
                    break;
                case 135:
                    maxDriveSpeedX = maxDriveSpeed;
                    maxDriveSpeedY = -maxDriveSpeed;
                    distance = -distance;
                    break;
                case -135:
                    maxDriveSpeedX = maxDriveSpeed;
                    maxDriveSpeedY = maxDriveSpeed;
                    distance = -distance;
                    break;
                default:
                    maxDriveSpeedX = 0;
                    maxDriveSpeedY = 0;
            }
            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH * 2);
            setTargetPosition(moveCounts);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Start driving straight, and then enter the control loop
            driveRobot(maxDriveSpeedX, maxDriveSpeedY, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                driveRobot(maxDriveSpeedX, maxDriveSpeedY, -turnSpeed);
                //                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    @Override
    public RobotHardware moveDiagonally(double distance, int angle) {
        return driveDiagonal(0.7, distance, angle, getHeading());
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    @Override
    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }

    @Override
    public RobotHardware stretchArm() {
        return setArmPower(0.75)
                .sleep(1000)
                .setArmPower(0)
                .setDumpPosition(0.26)
                .sleep(450)
                .setArmPower(-0.75)
                .sleep(650)
                .setArmPower(0);
    }

    @Override
    public RobotHardware resetArm() {
        setArmPower(0.75)
                .sleep(750)
                .setArmPower(0);
        for (double dumpPosition = 0.3; dumpPosition <= 0.8; dumpPosition += 0.025) {
            setDumpPosition(dumpPosition);
            sleep(70);
        }
        setDumpPosition(0.84);
        setArmPower(-0.75)
                .sleep(650)
                .setArmPower(0);
        return this;
    }

    //The functions below are unfinished
    //Don't use it!!!

    private void setDirectTargetPosition(int moveCountsX, int moveCountsY) {
        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + moveCountsX + moveCountsY);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + moveCountsX - moveCountsY);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + moveCountsX - moveCountsY);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + moveCountsX + moveCountsY);
    }

    private RobotHardware driveDirect(double maxDriveSpeed,
                                      double distanceX,
                                      double distanceY,
                                      double heading) {
        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {
            double angle = Math.atan2(distanceY, distanceX);
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            double maxDriveSpeedX = maxDriveSpeed * Math.cos(angle);
            double maxDriveSpeedY = maxDriveSpeed * Math.sin(angle);
            // Determine new target position, and pass to motor controller
            int moveCountsX = (int) (distanceX * COUNTS_PER_INCH);
            int moveCountsY = (int) (distanceY * COUNTS_PER_INCH);
            setDirectTargetPosition(moveCountsX, moveCountsY);

            setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start driving straight, and then enter the control loop
            driveRobot(maxDriveSpeedX, maxDriveSpeedY, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() && isAllBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distanceX < 0 && distanceY < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                driveRobot(maxDriveSpeedX, maxDriveSpeedY, -turnSpeed);
                //                telemetry.addData("x","%4.2f, %4.2f, %4.2f, %4.2f, %4d",maxDriveSpeed,distance,heading,turnSpeed,moveCounts);
                telemetry.update();
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            stopMotor();
            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return this;
    }

    @Override
    public RobotHardware moveDirect(double x, double y, double h) {
        double[] currentPos = {getPosition().x, getPosition().y, getPosition().h};
        double[] displacement = getDisplacement(currentPos, new double[]{x, y, h});
        return driveDirect(0.7, displacement[0], displacement[1], getHeading());
    }

    @Override
    public <T extends CameraControl> T getCameraControl(Class<T> controlType) {
        if (visionPortal == null) return null;
        return visionPortal.getCameraControl(controlType);
    }

    @Override
    public VisionPortal.CameraState getCameraState() {
        if (visionPortal == null) return null;
        return visionPortal.getCameraState();
    }
}