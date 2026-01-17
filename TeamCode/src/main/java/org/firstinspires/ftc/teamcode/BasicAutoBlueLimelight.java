/* Copyright (c) 2023 FIRST. All rights reserved.
 FIRST Inspires FTC
 Original function ality of robot aligns itstelf with April tag

 Reorganize code into a State Machine Arictecture

 Remove some comments to mitigate congnative overload
 Make lines fit on one printed line and leverage white space for read
 ability
 
 WARNING - prototype code
 This code is not complete, in its final form, or adequately tested
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import java.util.concurrent.TimeUnit;

import java.util.List;
/*
 * This sample assumes that the current game AprilTag Library (usually for the
 * current season) is being loaded by default, so you should choose to approach
 * a valid tag ID.
 */

@Autonomous(name = "BasicAutoBlue Version 20260105 LL", group = "Concept")
//@Disabled
public class BasicAutoBlueLimelight extends LinearOpMode {
    // Adjustable robot setting.

    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx launchMotor = null;
    private DcMotorEx intakeMotor = null;
    private Servo launchServo = null;
   // private static final boolean USE_WEBCAM = true;  // false for a phone camera
    private static final int BLUEGOALTAG = 20;
    private static final int GPPTAG = 21;
    private static final int PGPTAG = 22;
    private static final int PPGTAG = 23;// -1 for ANY tag.
    private static final double MIN_POS= 0.0;
    private static final double LAUNCH_POS= 0.4;
//    private VisionPortal visionPortal;               // Used to manage the video source.
//    private AprilTagProcessor aprilTag;
//    private AprilTagDetection goalTag = null;

    private double tagBearing;
    private int ApiriltagFoundid = 0;
    private boolean shooting = false;

    // Define the states 
    enum movebaby {
        STRAFE,
        READ_APRIL,
        TURN_TO_SHOOT,
        SHOOT_BALLS,
        MOTOR_TEST,
        STOP_ROBOT,
        LOOK_FOR_APRILTAG,
        READ_GOAL_APRIL,
        PARKING,
        LIMELIGHT_Test
    }
    // Set your start state here:

    //movebaby myRobotState  = movebaby.MOVE_TO_APRIL;
    //movebaby myRobotState  = movebaby.MOTOR_TEST;
    movebaby myRobotState = movebaby.STRAFE;
    //movebaby myRobotState = movebaby.LIMELIGHT_Test;
    // Manuafacture of the robot strucutural components
    enum manufacture {
        REV,
        GOBILDA
    }

    // Choose which robot system is in use, REV or Gobilds
    manufacture myBrand = manufacture.GOBILDA;
    // manufacture myBrand = manufacture.REV;

    //****** runOpMode ***********************************************************
    // Execution begins here
    Limelight3A limelight;
    
    @Override
    public void runOpMode() {
        // Initialize the Apriltag Detection process
        //initAprilTag();
		
        // Map the configuration motor labels to code variables
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        launchServo = hardwareMap.get(Servo.class, "launch_servo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        limelight.start();
		
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        switch (myBrand) {
            case GOBILDA:
                telemetry.addData("GoBILDA frame", "");
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
                launchMotor.setDirection(DcMotor.Direction.FORWARD);
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
        sleep(500); // provide time to read the brand of the frame


        launchServo.setPosition(MIN_POS);

//        if (USE_WEBCAM)
//            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            switch (myRobotState) {
                case SHOOT_BALLS:
                    shootBalls();
                    break;
                case STOP_ROBOT:
                    printState("STOP_ROBOT");
                    shutDown();
                    break;
                case READ_APRIL:
                    detectObeliskApril(ApiriltagFoundid);
                    break;
                case TURN_TO_SHOOT:
                    printState("TURN_TO_SHOOT");
                    detectGoalApril(ApiriltagFoundid);
                    break;
                case STRAFE:
                    printState("STRAFE");
                    strafe(2000);
                    break;
                case MOTOR_TEST:
                    testMotorsBasic(0.2);
                    break;
                case LOOK_FOR_APRILTAG:
                    printState("LOOK_FOR_APRILTAG");
                    lookForAprilTag(0.3f);
                    break;
                case PARKING:
                    parking();
                    break;
                case LIMELIGHT_Test:
                    limelightTest();
                    break;
            }
        }
    }


    //****** strafe State **************************************************

    public void strafe(int strafeTime) {

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        moveRobot(0, -0.6, 0);

        // Let the motors run until the preset time is reached
        while (runtime.milliseconds() < strafeTime) {

            // Display position - can be removed
            telemetry.addData("  left  position ", leftFrontDrive.getCurrentPosition());
            telemetry.addData("  right position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("  left  velocity ", leftFrontDrive.getVelocity());
            telemetry.addData("  right velocity", rightFrontDrive.getVelocity());
            telemetry.update();
        }
        moveRobot(0, -0, 0);
        myRobotState = movebaby.LOOK_FOR_APRILTAG;
    }


    //********************************************************************************
    public void lookForAprilTag(float lookingSpeed) {
        boolean Nodetection = true;

        LLResult result = limelight.getLatestResult();
        if(result != null  && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                ApiriltagFoundid = fr.getFiducialId();
                tagBearing = fr.getTargetXDegrees();
                Nodetection = false;
                telemetry.addData("  TAG ID  ", ApiriltagFoundid);
                telemetry.update();
                sleep(1000);
            }
        }

        if (Nodetection) {
            turnslightly();
            myRobotState = movebaby.LOOK_FOR_APRILTAG;
        } else {

            if (shooting) {
                myRobotState = movebaby.TURN_TO_SHOOT;
            } else {
                myRobotState = movebaby.READ_APRIL;
            }


        }
    }


    //********************************************************************************
    public void turnslightly() {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        moveRobot(0, 0, 0.5);
        while (runtime.milliseconds() < 100) {

            telemetry.addData("  left  position ", leftFrontDrive.getCurrentPosition());
            telemetry.addData("  right position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("  left  velocity ", leftFrontDrive.getVelocity());
            telemetry.addData("  right velocity", rightFrontDrive.getVelocity());
            telemetry.update();
        }
        moveRobot(0, 0, 0);
        sleep(500);

    }


    // *************************** detectObeliskApril ******************************
    //

    public void detectObeliskApril(int ApiriltagFound) {

        if ((ApiriltagFound == GPPTAG) ||
                (ApiriltagFound == PGPTAG) ||
                (ApiriltagFound == PPGTAG)) {

            myRobotState = movebaby.TURN_TO_SHOOT;
        } else {
            turnslightly();
            myRobotState = movebaby.LOOK_FOR_APRILTAG;
        }
    }


//*********************************  parking  ********************************

    public void parking() {
        moveRobot(0, 0, 0);
        sleep(100);
        moveRobot(-0.6, 0, 0);
        sleep(500);
        moveRobot(0, 0, 0);
        myRobotState = movebaby.STOP_ROBOT;

    }


    //****** turntoShoot State ************************************************
    //
    public void detectGoalApril(int ApiriltagFound) {
        shooting = true;

        if ((ApiriltagFound == BLUEGOALTAG)) {
            myRobotState = movebaby.SHOOT_BALLS;
        } else {
            turnslightly();
            myRobotState = movebaby.LOOK_FOR_APRILTAG;
        }
        sleep(500);
    }


    //****** shootBalls State **************************************************
    public void shootBalls() {
        launchMotor.setPower(0.9);
        centerTag();
        telemetry.addData("bearing ",  tagBearing);
        telemetry.addData(">", "shootballs");
        telemetry.update();
        sleep(2000);
        launchServo.setPosition(LAUNCH_POS);
        sleep(500);
        launchServo.setPosition(MIN_POS);

        launchMotor.setPower(0.0);
        // put code to launch artifact here
        myRobotState = movebaby.PARKING;
    }


    //****** centerTag State **************************************************
    public void centerTag() {

        boolean Nodetection = true;
        while (tagBearing < -1.0)  {
            telemetry.addData("bearing ", tagBearing);
            telemetry.update();
            turnslightly();
            LLResult result = limelight.getLatestResult();
            if(result != null  && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    ApiriltagFoundid = fr.getFiducialId();
                    tagBearing = fr.getTargetXDegrees();
                    Nodetection = false;
                    telemetry.addData("  TAG ID  ", ApiriltagFoundid);
                    telemetry.update();
                    //sleep(1000);
                }
            }

            sleep(500);
        }
    }




    //****** testMotorBasic State **********************************************
    // Epp
    // Make sure that the direction of all four Mecanum wheels are set up
    // correctly. They should turn forward.
    public void testMotorsBasic(double power) {
        // Send powers to the wheels.
        telemetry.addData(">", "leftFront");
        telemetry.update();
        leftFrontDrive.setPower(power);
        sleep(2000);
        leftFrontDrive.setPower(0);
        telemetry.addData(">", "rightFront");
        telemetry.update();
        rightFrontDrive.setPower(power);
        sleep(2000);
        rightFrontDrive.setPower(0);
        telemetry.addData(">", "leftBack");
        telemetry.update();
        leftBackDrive.setPower(power);
        sleep(2000);
        leftBackDrive.setPower(0);
        telemetry.addData(">", "rightBack");
        telemetry.update();
        rightBackDrive.setPower(power);
        sleep(2000);
        rightBackDrive.setPower(0);
    }


    //********************** moveRobot *********************************************
    // Copyright (c) 2023 FIRST. All rights reserved.
    // From original April tag program
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


    //******************** printState Sate *************************************//
    // For Debug
    public void printState(String stateToPrint) {
        telemetry.addData("State is %s", stateToPrint);
        telemetry.update();
        //sleep(1000);
    }


    //******************* initAprilTag *******************************************
    // Copyright (c) 2023 FIRST. All rights reserved.
    // From original April tag program
    // Initialize the AprilTag processor.
    /*private void initAprilTag() {
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
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }


    //******************* setManualExposure *******************************************
    // Copyright (c) 2023 FIRST. All rights reserved.
    // From original April tag program
    //
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
    }*/


    // ***********shutdown State***************************************************
    // Epp
    // Turn wheel power off

    public void shutDown() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        launchMotor.setPower(0);
    }



    //**********************************  limelightTest ******************************************
    // program to prove tag ID and bearing can be obtained from Limelight
    // It looks like the ID isn't always read.  Maybe need to verify device is ready?
    public void limelightTest() {
        int AprilTagID = 0;


        LLResult result = limelight.getLatestResult();
        if(result != null  && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                AprilTagID = fr.getFiducialId();
                tagBearing = fr.getTargetXDegrees();

            }
        }

        telemetry.addData("TagID", AprilTagID);
        telemetry.addData("TagBearing", tagBearing);
        telemetry.update();
    }
}
