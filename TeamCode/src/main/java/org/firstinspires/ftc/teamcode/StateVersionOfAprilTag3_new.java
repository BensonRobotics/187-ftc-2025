/* Copyright (c) 2023 FIRST. All rights reserved.
 FIRST Inspires FTC
 Original function ality of robot aligns itstelf with April tag
 
 Modifications Ed C. Epp 2024 to 2025
 Reorganize code into a State Machine Arictecture
 Add move to a position state
 Remove some comments to mitigate congnative overload
 Make lines fit on one printed line and leverage white space for read
 ability
 
 WARNING - prototype code
 This code is not complete, in its final form, or adequately tested
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This sample assumes that the current game AprilTag Library (usually for the
 * current season) is being loaded by default, so you should choose to approach
 * a valid tag ID.
 *
 * The robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use
 *    the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly
 *    in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range
 *    to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN
 * and TURN_GAIN constants.
 */

@Autonomous(name="April Tag State Version 20251008", group = "Concept")

public class StateVersionOfAprilTag3_new extends LinearOpMode {
    // Adjustable robot setting.
    final double DESIRED_DISTANCE = 12.0; // how close the camera should get to 
    // the target (inches)
    //  Set the GAIN constants to control the relationship between the measured 
    //  position error, and how much power is applied to the drive motors to 
    //  correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, 
    //  or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   // Forward Speed Control "Gain". e.g.
    // Ramp up to 50% power at a 25 inch error.
    // (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;  //  Strafe Speed Control "Gain".  e.g. 
    // Ramp up to 37% power at a 25 degree
    // Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up
    //to 25% power at a 25 degree error.
    // (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5; //  Clip the approach speed to this max 
    //value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;//  Clip the strafing speed to this max 
    //value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;  //  Clip the turn speed to this max value 
    //(adjust for your robot)

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx intake = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx launcher = null;
   //private static final boolean USE_WEBCAM = false;  // false for a phone camera
    private static final int DESIRED_TAG_ID = 16;    // -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the 
    // AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a 

    //detected AprilTag

    boolean isOnLeft = true;
    boolean isOnback = true;

    boolean targetFound = false;    // AprilTag target is detected
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    // for move a distance
    final int DISTANCE = 2000; // number of clicks to move
    final int VELOCITY = 600; // number of clicks per second
    final double POWER = 0.5; // percent of full power
    IMU imu;

    // Define the states 
    enum movebaby {
        MOTOR_TEST,
        MOTOR_TEST_BASIC,
        MOTOR_TO_POSITION,
        POWER_TO_POSITION,
        FIND_APRIL,
        MOVE_TO_APRIL,
        TURN_45,
        STOP_ROBOT,
        TURN_ANGLE,
        BASIC_AUTO,
        AUTO_ONE_BALL,

        LIMELIGHT_TEST
    }


    // Initial state
    //movebaby myRobotState  = movebaby.MOVE_TO_APRIL;
    // movebaby myRobotState  = movebaby.TURN_45;
   //movebaby myRobotState = movebaby.AUTO_ONE_BALL;
    movebaby myRobotState = movebaby.LIMELIGHT_TEST;


    //movebaby myRobotState  = movebaby.MOTOR_TEST;
    // movebaby myRobotState  = movebaby.MOTOR_TEST_BASIC;
    // movebaby myRobotState  = movebaby.MOTOR_TO_POSITION;
    //movebaby myRobotState  = movebaby.POWER_TO_POSITION;
    //movebaby myRobotState  = movebaby.TURN_ANGLE;
    // Manuafacture of the robot strucutural components
    enum manufacture {
        REV,
        GOBILDA
    }

    manufacture myBrand = manufacture.REV;
   // manufacture myBrand = manufacture.GOBILDA;

    //****** runOpMode ***********************************************************
    // Execution begines here

    @Override
    public void runOpMode() {
        // Initialize the Apriltag Detection process



        while (isStarted() == false){




            if(gamepad1.b){
                isOnLeft=true;
            }
            else if (gamepad1.x){
                isOnLeft = false;
            }

            if(gamepad1.a){
                isOnback=false;
            }
            else if (gamepad1.y){
                isOnback = false;
            }
            telemetry.addData("is On back:", isOnback);
            telemetry.addData("is On Left:", isOnLeft);
            telemetry.update();

        }


        initAprilTag();


        // Map the configuration motor labels to code variables
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launch_motor");
        intake = hardwareMap.get(DcMotorEx.class, "intake_motor");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD

        ));
        imu.initialize(parameters);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        switch (myBrand) {
            case GOBILDA:
                telemetry.addData("GoBILDA frame", "");
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
                break;
            case REV:
                telemetry.addData("REV frame", "");
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
                break;
            default:
        }
        telemetry.update();
        sleep(3000); // provide time to read the brand of the frame


       /* if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
*/
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;


            switch (myRobotState) {
                case MOVE_TO_APRIL:
                    moveToAprilTag();  // uses camera to find April Tag
                    break;
                case STOP_ROBOT:
                    shutDown();
                    break;
                case FIND_APRIL:
                    detectApril();
                    break;
                case POWER_TO_POSITION:
                    powerToPosition(DISTANCE);
                    break;
                case MOTOR_TO_POSITION:
                    motorToPosition(DISTANCE);
                    break;
                case TURN_ANGLE:
                    turnAngle(10);

                    break;
                case TURN_45:
                    turn45UsingIMU(45.0);
                    break;
                case MOTOR_TEST:
                    testMotors();
                    break;

                case MOTOR_TEST_BASIC:
                    testMotorsBasic(0.5);
                    break;

                case AUTO_ONE_BALL:
                    autoOneBall();
                    break;
                case LIMELIGHT_TEST:
                    limelightTest();
                    break;
                default:
                    shutDown();
            }

            telemetry.update();
        }
        sleep(10);
    }


    //****** powerToPosition State **************************************************
    // Epp
    //
    // distance: number of clicks to move forward or backword
    // "0" position is redefined to current position
    // User power instead of speed

    public void powerToPosition(int distance) {

        // The current position is set to zero

        // Left side
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(DISTANCE); // do before next step
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Seems to overload the cpu so drop check for back wheels.
        // Dig deeper

        //leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftBackDrive.setTargetPosition(DISTANCE); // do before next step
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // right side
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition(DISTANCE);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightBackDrive.setTargetPosition(DISTANCE);
        //rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Fire up the motors
        leftFrontDrive.setPower(POWER);
        leftBackDrive.setPower(POWER);
        rightFrontDrive.setPower(POWER);
        rightBackDrive.setPower(POWER);

        // Let the motors run until the preset count is reached
        while (opModeIsActive() && (leftFrontDrive.isBusy() ||
                rightFrontDrive.isBusy())) {
            // Display position - can be removed
            telemetry.addData("  left  position ", leftFrontDrive.getCurrentPosition());
            telemetry.addData("  right position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("  left  power ", leftFrontDrive.getPower());
            telemetry.addData("  right power", rightFrontDrive.getPower());
            telemetry.update();
        }

        myRobotState = movebaby.STOP_ROBOT;
    }

    //****** motorToPosition State **************************************************
    // Epp
    //
    // distance: number of clicks to move forward or backword
    // "0" position is redefined to current position

    public void motorToPosition(int distance) {

        // The current position is set to zero

        // Left side
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(DISTANCE); // do before next step
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Seems to overload the cpu so drop check for back wheels.
        // Dig deeper

        //leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftBackDrive.setTargetPosition(DISTANCE); // do before next step
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // right side
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition(DISTANCE);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightBackDrive.setTargetPosition(DISTANCE);
        //rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Fire up the motors
        leftFrontDrive.setVelocity(VELOCITY);
        leftBackDrive.setVelocity(VELOCITY);
        rightFrontDrive.setVelocity(VELOCITY);
        rightBackDrive.setVelocity(VELOCITY);

        // Let the motors run until the preset count is reached
        while (opModeIsActive() && leftFrontDrive.isBusy()) {
            // Display position - can be removed
            telemetry.addData("  left  position ", leftFrontDrive.getCurrentPosition());
            telemetry.addData("  right position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("  left  velocity ", leftFrontDrive.getVelocity());
            telemetry.addData("  right velocity", rightFrontDrive.getVelocity());
            telemetry.update();
        }

        myRobotState = movebaby.STOP_ROBOT;
    }

    //****** detectApril State **************************************************
    //
    // Not implemented

    public void detectApril() {

    }

    public void StrafeUsingImu(double langth) {
        {
            moveRobot(0.0, 0.0, 0.0);
        }
    }

    //****** turn45UsingIMU State *********************************************
    //
    public void turn45UsingIMU(double angle) {

        while (opModeIsActive()) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("yaw", botHeading);
            telemetry.update();
            sleep(10);

            if (angle < imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 5 &&
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - 5 < angle) {

                moveRobot(0.0, 0.0, 0.0);
            } else {
                moveRobot(0.0, 0.0, (angle - (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))) / 45);


            }
            /*
            if ( botHeading +0.1 > 100*angle || botHeading - 0.1 < 100*angle ) {
                moveRobot(0.0,0.0,1.0);
               //leftFrontDrive.setPower(1);
               // rightFrontDrive.setPower(-1);
               // rightBackDrive.setPower(-1);
                //leftBackDrive.setPower(1);


                while (100*angle == imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 0.1 ||
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - 0.1 > 100*angle);
                {telemetry.addData("yaw", botHeading);}
                moveRobot(0.0,0.0,0.0);
              //  leftFrontDrive.setPower(0);
               // rightFrontDrive.setPower(0);
               // rightBackDrive.setPower(0);
               // leftBackDrive.setPower(0);
            }else { moveRobot(0.0,0.0,0.0);
               // leftFrontDrive.setPower(0);
              //  rightFrontDrive.setPower(0);
               // rightBackDrive.setPower(0);
               // leftBackDrive.setPower(0);
            }
                */
        }
    }


//************************************************************************

    public void turnAngle(int angle) {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(22 * angle); // do before next step
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition(22 * angle);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(-1);
        rightBackDrive.setPower(-1);
        leftBackDrive.setPower(1);

        // Let the motors run until the preset count is reached
        while (opModeIsActive() &&
                (leftFrontDrive.isBusy() || rightFrontDrive.isBusy())) {
            telemetry.addData(" left position ", leftFrontDrive.getCurrentPosition());
            telemetry.addData(" right position ", rightFrontDrive.getCurrentPosition());
            // telemetry.addData(" left velocity ", leftFrontDrive.getVelocityPosition());
            // telemetry.addData(" right velocity ", rightFrontDrive.getVelocityPosition());
        }
        myRobotState = movebaby.STOP_ROBOT;
    }


    public void testMotorsBasic(double power) {
        // Send powers to the wheels.
        leftFrontDrive.setPower(power);
        sleep(2000);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(power);
        sleep(2000);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(power);
        sleep(2000);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(power);
        sleep(2000);
        ;
        leftBackDrive.setPower(0);
    }


    //***** rightFrontDrive.setPower(0);    * testMotors State ************************************************
    // Epp   sleep(2000);
    // Test 3 robot motions

    public void testMotors() {
        // Forward and Reverse
        moveRobot(0.5, 0.0, 0.0);
        sleep(2000);
        moveRobot(-0.5, 0.0, 0.0);
        sleep(2000);
        moveRobot(0.0, 0.0, 0.0);
        sleep(2000);

        // Strafe
        moveRobot(0.0, 0.5, 0.0);
        sleep(2000);
        moveRobot(0.0, -0.5, 0.0);
        sleep(2000);
        moveRobot(0.0, 0.0, 0.0);
        sleep(2000);

        // Turn
        moveRobot(0.0, 0.0, 0.5);
        sleep(2000);
        moveRobot(0.0, 0.0, -0.5);
        sleep(2000);
        moveRobot(0.0, 0.0, 0.0);
        sleep(2000);

        myRobotState = movebaby.STOP_ROBOT;
    }

    // ***********shutdown State***************************************************
    // Epp
    // Turn wheel power off

    public void shutDown() {
        //printState();  // for debug
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    // *************************** moveToAprilTage ******************************
    // Copyright (c) 2023 FIRST. All rights reserved.
    // From original April tag program
    // robot aligns itstelf with April tag

    public void moveToAprilTag() {
        //sleep(10);
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        //telemetry.addData(">", "Loaded currentDetections");
        //telemetry.update();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == DESIRED_TAG_ID) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track 
                    // it right now
                    // add in targetFound = false;

                    telemetry.addData("Skipping", "Tag ID %d is not desired",
                            detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough 
                // information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary",
                        detection.id);
            }
        }
        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)",
                    desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches",
                    desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees",
                    desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees",
                    desiredTag.ftcPose.yaw);

            // Determine heading, range and Yaw (tag image rotation) error so we 
            // can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the 
            // robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                    drive, strafe, turn);

            // Apply desired axes motions to the drivetrain.
            if (Math.abs(rangeError) < 3) myRobotState = movebaby.STOP_ROBOT;

            else moveRobot(drive, strafe, turn);
        }

    }


    //******************** printState Sate *************************************//
    // Delacy
    public void printState() {
        switch (myRobotState) {
            case MOVE_TO_APRIL:
                telemetry.addData("State is ", "MOVE_TO_APRIL");
                break;
            case STOP_ROBOT:
                telemetry.addData("State is ", "STOP_ROBOT");
        }
        sleep(1500);
    }


    //********************** moveRobot *********************************************
    // Copyright (c) 2023 FIRST. All rights reserved.
    // From original April tag program
    // robot aligns itself with April tag
    // Move robot according to desired axes motions
    // <p>
    // Positive X is forward
    // <p>
    // Positive Y is strafe left
    // <p>
    // Positive Yaw is counter-clockwise

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    //******************* initAprilTag *******************************************
    // Copyright (c) 2023 FIRST. All rights reserved.
    // From original April tag program
    // robot aligns itstelf with April tag
    // Initialize the AprilTag processor.
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
       /* if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }*/
    }

    //******************* setManualExposure *******************************************
    // Copyright (c) 2023 FIRST. All rights reserved.
    // From original April tag program
    // robot aligns itstelf with April tag
    //
    // Manually set the camera gain and exposure.
    // This can only be called AFTER calling initAprilTag(), and only 
    // works for Webcams;

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() !=
                    VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl =
                    visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    //************* autoOneBall ****************
    public void  autoOneBall() {


        if(isOnback){
            if(isOnLeft){

            leftFrontDrive.setPower(-1);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(-1);
            leftBackDrive.setPower(0);
                launcher.setPower(0.85);
            sleep(1700);

                leftFrontDrive.setPower(1);
                rightFrontDrive.setPower(-1);
                rightBackDrive.setPower(-1);
                leftBackDrive.setPower(1);
            sleep(100);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);

            sleep(5000);

            intake .setPower(1.0);
            sleep(10000);


            launcher.setPower(0);
                intake .setPower(0);
            sleep(500);

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);

            //stop robot

            sleep(5);
            myRobotState = movebaby.STOP_ROBOT ;

            }
            else {
                leftFrontDrive.setPower(1);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(-1);
                leftBackDrive.setPower(0);
                launcher.setPower(0.85 );
                sleep(1000);

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);

                sleep(5000);

                intake .setPower(1.0);

                sleep(10000);


                launcher.setPower(0);

                sleep(500);

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);

                //stop robot

                sleep(5);
                myRobotState = movebaby.STOP_ROBOT ;
            }

        }else {


        if(isOnLeft){

        leftFrontDrive.setPower(-0.5);
        rightFrontDrive.setPower(-0.5);
        rightBackDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
            launcher.setPower(0.85);

        //move robot backword

        sleep(1000);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);

            sleep(4000);

        intake .setPower(1.0);

        sleep(500);


       // stop and launch ball

        sleep(5000);

        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(-1);
        leftBackDrive.setPower(0);
            launcher.setPower(0);
            intake .setPower(0);
            //move out of launch zone

            sleep(500);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);

            //stop robot

            sleep(5);
            myRobotState = movebaby.STOP_ROBOT ;
            }
                 else
            {
                leftFrontDrive.setPower(-0.5);
                rightFrontDrive.setPower(-0.5);
                rightBackDrive.setPower(-0.5);
                leftBackDrive.setPower(-0.5);
                launcher.setPower(0.85);

                //move robot backword

                sleep(1000);

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);

                sleep(4000);

                intake .setPower(1.0);

                sleep(500);


                // stop and launch ball

                sleep(5000);

                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(-1);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(-1);
                launcher.setPower(0);
                intake .setPower(0);
                //move out of launch zone

                sleep(500);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);

                //stop robot

                sleep(5);
                myRobotState = movebaby.STOP_ROBOT ;
            }
     }
    }

    //******************* limelightTest ***************


    public void limelightTest() {}

}




