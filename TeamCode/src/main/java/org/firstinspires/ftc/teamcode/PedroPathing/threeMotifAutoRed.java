

package org.firstinspires.ftc.teamcode.PedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.configVars.AUTOREVOLVERSPEED;
import static org.firstinspires.ftc.teamcode.configVars.AUTOVELOCITYMULT;
import static org.firstinspires.ftc.teamcode.configVars.GREENHIGHTHRESHOLD;
import static org.firstinspires.ftc.teamcode.configVars.GREENLOWTHRESHOLD;
import static org.firstinspires.ftc.teamcode.configVars.GREENLOWTHRESHOLDAUTO;
import static org.firstinspires.ftc.teamcode.configVars.LAUNCHSERVOFINAL;
import static org.firstinspires.ftc.teamcode.configVars.LAUNCHSERVOINIT;
import static org.firstinspires.ftc.teamcode.configVars.PURPLEHIGHTHRESHOLD;
import static org.firstinspires.ftc.teamcode.configVars.PURPLEHIGHTHRESHOLDAUTO;
import static org.firstinspires.ftc.teamcode.configVars.PURPLELOWTHRESHOLD;
import static org.firstinspires.ftc.teamcode.configVars.PURPLELOWTHRESHOLDAUTO;
import static org.firstinspires.ftc.teamcode.configVars.SERVOTIMERTHRESHOLD;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;


@Autonomous(name = "PedroPathing 3 Motif Auto Red")

public class threeMotifAutoRed extends OpMode {
    //id 21 is gpp
    //id 22 is pgp
    //id 23 is ppg
    int id = 0;

    //0 is empty, 1 is purple, 2 is green
    //indexes start at the intake and add 1 clockwise
    int[] intakeIndexColors = {1, 1, 2};

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, servoTimer, revolverTimer;

    int TPS = 2800;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeMotor = null;
    private NormalizedColorSensor colorSensor = null;
    private DcMotorEx revolverMotor = null;
    private Servo launcherServo;
    private DcMotorEx launchMotor = null;

    //data for the path
    private final Pose startPose = new Pose(116.38935507072645, 132.4926876048909, Math.toRadians(125));
    private final Pose detectMotifPose = new Pose(92.976744186, 111.9767441860465, Math.toRadians(125));
    private final Pose launchMotifPose = new Pose(99.76744186, 118.3953488372093, Math.toRadians(35));
    private final Pose MotifOnePose = new Pose(97.1395348837, 88.30232558139534, Math.toRadians(180));

    private final Pose MotifOneControl = new Pose(65.4534883721, 103.34883720930232);
    private final Pose IntakeMotifOnePose = new Pose(121.534883721, 83.30232558139534, Math.toRadians(0));

    private final Pose MotifTwoControl = new Pose(65.4534883721, 90.73255813953489);

    private final Pose MotifTwoPose = new Pose(92.1395348837, 60.06976744186046, Math.toRadians(0));
    private final Pose IntakeMotifTwoPose = new Pose(126.534883721, 60.06976744186046, Math.toRadians(0));
    private int pathState;
    private double revolverTargetPosition = 0;

    private boolean isRevolverPositionSetYet = false;
    private boolean isServoPositionSetYet = true;
    private boolean isServoUpYet = false;
    private boolean isServoDownYet = false;

    private boolean isLaunchDoneYet = false;

    private boolean isRevolverTurnedYet = false;

    private boolean isRevolverFirstCheck = false;
    Limelight3A limelight;
    /*
        motif 1 is ppg
        motif 2 is pgp
        motif 3 is gpp
     */

    final float[] hsvValues = new float[3];

    int launchedArtifacts = 0;
    private Path scorePreload;
    private PathChain launchMotifPreload, prepareMotifOneIntake, intakeMotifOne, launchMotifOne, prepareMotifTwoIntake, intakeMotifTwo, launchMotifTwo;


    private void setPathState(int i) {
        pathState = i;
    }
    //builds paths
    public void buildPaths() {

        //path between start position and detecting motif
        scorePreload = new Path(new BezierLine(startPose, detectMotifPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), detectMotifPose.getHeading());

        //moves to launch the first motif
        launchMotifPreload = follower.pathBuilder()
                .addPath(new BezierLine(detectMotifPose, launchMotifPose))
                .setLinearHeadingInterpolation(detectMotifPose.getHeading(), launchMotifPose.getHeading())
                .build();

        //moves to intake the first motif
        prepareMotifOneIntake = follower.pathBuilder()
                .addPath(new BezierCurve(launchMotifPose, MotifOneControl, MotifOnePose))
                .setLinearHeadingInterpolation(launchMotifPose.getHeading(), MotifOnePose.getHeading())
                .build();

        //intakes first motif
        intakeMotifOne = follower.pathBuilder()
                .addPath(new BezierLine(MotifOnePose, IntakeMotifOnePose))
                .setLinearHeadingInterpolation(MotifOnePose.getHeading(), IntakeMotifOnePose.getHeading())
                .build();

        //moves to launch the first motif
        launchMotifOne = follower.pathBuilder()
                .addPath(new BezierLine(IntakeMotifOnePose, launchMotifPose))
                .setLinearHeadingInterpolation(IntakeMotifOnePose.getHeading(), launchMotifPose.getHeading())
                .build();

        prepareMotifTwoIntake = follower.pathBuilder()
                .addPath(new BezierCurve(launchMotifPose, MotifTwoControl, MotifTwoPose))
                .setLinearHeadingInterpolation(launchMotifPose.getHeading(), MotifTwoPose.getHeading())
                .build();

        intakeMotifTwo = follower.pathBuilder()
                .addPath(new BezierLine(MotifTwoPose, IntakeMotifTwoPose))
                .setLinearHeadingInterpolation(MotifOnePose.getHeading(), IntakeMotifTwoPose.getHeading())
                .build();

        launchMotifTwo = follower.pathBuilder()
                .addPath(new BezierLine(IntakeMotifTwoPose, launchMotifPose))
                .setLinearHeadingInterpolation(IntakeMotifTwoPose.getHeading(), launchMotifPose.getHeading())
                .build();


    }




    public void launchArtifact(){
        launchMotor.setVelocity(TPS * AUTOVELOCITYMULT);
        if (isServoPositionSetYet) {
            isServoPositionSetYet = false;
            launcherServo.setPosition(0.4);
            servoTimer.resetTimer();
        }
        else if (servoTimer.getElapsedTime() > SERVOTIMERTHRESHOLD && !isServoUpYet){
            servoTimer.resetTimer();
            isServoUpYet = true;
            launcherServo.setPosition(0);



        }

        else if (servoTimer.getElapsedTime() > SERVOTIMERTHRESHOLD && isServoUpYet){
            isServoDownYet = true;
            launchedArtifacts += 1;
            intakeIndexColors[2] = 0;


        }
        telemetry.addData("ServoPos", launcherServo.getPosition());
        telemetry.addData("isServoUpYetVar", isServoUpYet);
        telemetry.addData("isServoDownYetVar", isServoDownYet);
        telemetry.addData("isServoPositionYetVar", isServoPositionSetYet);

        telemetry.addData("intakeArray 0:", intakeIndexColors[0]);
        telemetry.addData("intakeArray 1:", intakeIndexColors[1]);
        telemetry.addData("intakeArray 2:", intakeIndexColors[2]);



        //telemetry.addData("servoTarget", launcherServo.pos);
        telemetry.update();

        //isLaunchDoneYet = true;
    }

    public void turnRevolverClockwise(){
        if (!isRevolverPositionSetYet) {
            revolverTargetPosition += 537.7 / 3;
            revolverMotor.setTargetPosition((int) revolverTargetPosition);
            revolverMotor.setPower(AUTOREVOLVERSPEED);
            intakeMotor.setPower(1);
            isRevolverPositionSetYet = true;
        }
        else if (isRevolverFirstCheck && revolverTimer.getElapsedTime() >1000 && revolverMotor.isBusy()){
            revolverTimer.resetTimer();
        }
        else if(!revolverMotor.isBusy()){
            if (!isRevolverFirstCheck) {
                revolverTimer.resetTimer();
                isRevolverFirstCheck = true;
            }
            if (isRevolverFirstCheck && revolverTimer.getElapsedTime() > 1000) {
                isRevolverTurnedYet = true;
                shiftIntakeArrayRight();
            }
        }

    }

    public void turnRevolverCounterClockwise(){
        if (!isRevolverPositionSetYet) {
            revolverTargetPosition -= 537.7 / 3;
            revolverMotor.setTargetPosition((int) revolverTargetPosition);
            revolverMotor.setPower(AUTOREVOLVERSPEED);
            intakeMotor.setPower(1);

            isRevolverPositionSetYet = true;
        }
        else if (isRevolverFirstCheck && revolverTimer.getElapsedTime() >1000 && revolverMotor.isBusy()){
            revolverTimer.resetTimer();
        }
        else if (!revolverMotor.isBusy()){
            if (!isRevolverFirstCheck) {
                revolverTimer.resetTimer();
                isRevolverFirstCheck = true;
            }
            if (isRevolverFirstCheck && revolverTimer.getElapsedTime() > 1500) {
                isRevolverTurnedYet = true;
                shiftIntakeArrayLeft();
            }
        }

    }

    //the threshold of launched artifacts where it searches for a green
    //0 for gpp
    //1 for pgp
    //2 for ppg

    public void resetLaunchVars(){
        isServoDownYet = false;
        isServoUpYet = false;
        isLaunchDoneYet = false;
        isRevolverTurnedYet = false;
        isRevolverPositionSetYet = false;
        isServoPositionSetYet = true;
        isRevolverFirstCheck = false;
    }
    public void launchMotifShell(int greenThreshold){
        telemetry.addData("launchedArtifacts", launchedArtifacts);
        telemetry.addData("revolve power", revolverMotor.getPower());
        telemetry.addData("revolve tgt pos", revolverMotor.getTargetPosition());
        telemetry.addData("intakeArray 0:", intakeIndexColors[0]);
        telemetry.addData("intakeArray 1:", intakeIndexColors[1]);
        telemetry.addData("intakeArray 2:", intakeIndexColors[2]);


        telemetry.update();

        //launches purple
        if (launchedArtifacts != greenThreshold) {
            if (intakeIndexColors[2] == 1) {
                if (!isServoDownYet) {
                    launchArtifact();
                }
                else{
                    resetLaunchVars();
                }

            } else if (intakeIndexColors[1] == 1) {
                if (!isRevolverTurnedYet){
                    turnRevolverClockwise();
                    //isRevolverTurnedYet = true;

                }
                else if (!isServoDownYet && isRevolverTurnedYet) {
                    launchArtifact();
                }
                else if (isServoDownYet && isRevolverTurnedYet){
                    resetLaunchVars();

                }

            } else if (intakeIndexColors[0] == 1) {
                if (!isRevolverTurnedYet){
                    turnRevolverCounterClockwise();
                    // isRevolverTurnedYet = true;

                }
                else if (!isServoDownYet && isRevolverTurnedYet) {
                    launchArtifact();
                }
                else if (isServoDownYet && isRevolverTurnedYet){
                    resetLaunchVars();

                }
            }
        }
        else if (launchedArtifacts == greenThreshold){

            if (intakeIndexColors[2] == 2) {
                if (!isServoDownYet) {
                    launchArtifact();
                }
                else{
                    resetLaunchVars();
                }


            }

            else if (intakeIndexColors[1] == 2){

                if (!isRevolverTurnedYet){
                    turnRevolverClockwise();
                    //isRevolverTurnedYet = true;

                }

                else if (!isServoDownYet && isRevolverTurnedYet) {
                    launchArtifact();
                }
                else if(isServoDownYet && isRevolverTurnedYet){
                    resetLaunchVars();
                }
            }


            else if (intakeIndexColors[0] == 2){
                if (!isRevolverTurnedYet){
                    turnRevolverCounterClockwise();
                    // isRevolverTurnedYet = true;

                }

                else if (!isServoDownYet && isRevolverTurnedYet) {
                    launchArtifact();
                }
                else if (isServoDownYet && isRevolverTurnedYet){
                    resetLaunchVars();
                }

            }


        }
        if (launchedArtifacts == 3){
            isLaunchDoneYet = true;
            launchedArtifacts = 0;
        }


    }
    public void launchMotif(int detected_id) {

        //int launchedArtifacts = 0;
        //purple purple green
        if (detected_id == 21) {
            launchMotifShell(0);
        }

        //pgp
        else if (detected_id == 22) {
            launchMotifShell(1);
        } else if (detected_id == 23) {
            launchMotifShell(2);
        } else {
            launchMotifShell(0);

        }
    }
    //launches the motif


    public void intakeMotif(){

        NormalizedRGBA colors= colorSensor.getNormalizedColors();
        colorSensor.setGain(3);
        intakeMotor.setPower(1);
        Color.colorToHSV(colors.toColor(), hsvValues);



        //  telemetry.addData("1",slot1);
        //  telemetry.addData("2",slot2);
        //  telemetry.addData("3",slot3);
        telemetry.addData("h",hsvValues[0]);
        telemetry.addData("v",hsvValues[1]);
        telemetry.addData("red",colors.red);
        telemetry.addData("green",colors.green);
        telemetry.addData("blue",colors.blue);


        if (hsvValues[0] > PURPLELOWTHRESHOLDAUTO && hsvValues[0] < PURPLEHIGHTHRESHOLDAUTO) {
            telemetry.addData("purple", 2);
            intakeIndexColors[0] = 1;
            if (!isRevolverTurnedYet) {
                turnRevolverCounterClockwise();
            }
            else if(isRevolverTurnedYet) {
                resetLaunchVars();
            }


        } else if (hsvValues[0] > GREENLOWTHRESHOLDAUTO && hsvValues[0] < GREENLOWTHRESHOLDAUTO) {

            telemetry.addData("green", 1);
            intakeIndexColors[0] = 2;
            if (!isRevolverTurnedYet) {
                turnRevolverCounterClockwise();
            }
            else if(isRevolverTurnedYet){
                resetLaunchVars();

            }


        } else {

            telemetry.addData("no ball", 0);
            //  RGB.setPosition(0.3);

        }
        /**/
        //set intake to run

    }

    public void limelightDetectTag(){

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);

            id = fiducial.getFiducialId(); // The ID number of the fiducial
            telemetry.addData("Fiducial: ", id);




        } else {
            telemetry.addData("Limelight", "No Targets");
        }

    }

    public void shiftIntakeArrayRight(){

        int zeroOld = intakeIndexColors[0];
        int oneOld = intakeIndexColors[1];
        int twoOld = intakeIndexColors[2];

        intakeIndexColors[0] = twoOld;
        intakeIndexColors[1] = zeroOld;
        intakeIndexColors[2] = oneOld;
    }

    public void shiftIntakeArrayLeft(){

        int zeroOld = intakeIndexColors[0];
        int oneOld = intakeIndexColors[1];
        int twoOld = intakeIndexColors[2];

        intakeIndexColors[0] = oneOld;
        intakeIndexColors[1] = twoOld;
        intakeIndexColors[2] = zeroOld;
    }
    //manages state machine switches
    public void autonomousPathUpdate() {

        //the state doesn't automatically wait for the path to finish
        switch(pathState){
            case 0:
                follower.followPath(scorePreload, true);

                setPathState(1);
                break;
            case 1:

                //detects motif
                limelightDetectTag();

                if (!follower.isBusy() && id != 0) {
                    follower.followPath(launchMotifPreload, true);
                    //if this doesnt work set heading constraint to low number
                    //follower.getCurrentPath().setLinearHeadingInterpolation(detectMotifPose.getHeading(), launchMotifPose.getHeading());
                    setPathState(2);
                }
                break;


            case 2:


                if (!follower.isBusy()) {
                    //launchMotif(id);
                    launchMotif(id);
                    if (isLaunchDoneYet) {
                        resetLaunchVars();
                        follower.followPath(prepareMotifOneIntake, true);
                        setPathState(3);
                    }
                }

                break;
            case 3:

                if (!follower.isBusy()) {
                    follower.followPath(intakeMotifOne, true);
                    follower.setMaxPower(0.15);

                    setPathState(4);
                }
                break;
            case 4:

                intakeMotif();
                if (!follower.isBusy()) {
                    follower.followPath(launchMotifOne, true);
                    follower.setMaxPower(1);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    launchMotif(id);
                    if (isLaunchDoneYet) {
                        resetLaunchVars();
                        follower.followPath(prepareMotifOneIntake, true);
                        setPathState(6);
                    }
                    //  setPathState(6);
                }
                break;


            case 6:

                if (!follower.isBusy()) {
                    follower.setMaxPower(0.15);

                    follower.followPath(intakeMotifTwo, true);
                    setPathState(7);
                }
                break;
            case 7:


                intakeMotif();
                if (!follower.isBusy()) {
                    follower.followPath(launchMotifTwo, true);

                    setPathState(8);
                    follower.setMaxPower(1);
                }
                break;
            case 8:
                if(!follower.isBusy()) {

                    launchMotif(id);
                    if (isLaunchDoneYet) {
                        resetLaunchVars();

                    }
                }
                break;



        }
    }
    @Override
    public void init() {


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        servoTimer = new Timer();
        revolverTimer = new Timer();
        opmodeTimer.resetTimer();

        resetLaunchVars();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        // launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherServo = hardwareMap.get(Servo.class, "launcher_servo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        revolverMotor = hardwareMap.get(DcMotorEx.class, "revolver_motor");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        isLaunchDoneYet = false;
        limelight.pipelineSwitch(4);
        int id = 0;
        revolverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        colorSensor = hardwareMap.get(NormalizedColorSensor .class,"sensor_color");
        launchedArtifacts = 0;
        follower = Constants.createFollower(hardwareMap);
        colorSensor.setGain(3);
        revolverMotor.setTargetPosition(0);
        revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        revolverMotor.setPower(AUTOREVOLVERSPEED);
        buildPaths();
        follower.setStartingPose(startPose);


    }
    @Override

    public void start() {
        launcherServo.setPosition(LAUNCHSERVOINIT);
        launchMotor.setVelocity(TPS * AUTOVELOCITYMULT);

        setPathState(0);
    }
    @Override

    public void loop() {
        //makes state machine run constantly
        autonomousPathUpdate();
        //detects if the robot is following the path
        follower.update();

        //private final Pose startPose = new Pose(28.5, 128, Math.toRadians(90)); // Start Pose of our robot.
        //private final Pose scorePose1 = new Pose(35.37, 119.3, Math.toRadians(145)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        //private final Pose leaveLaunchLine = new Pose(53, 57.8, Math.toRadians(145));
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("intakeArray 0:", intakeIndexColors[0]);
        telemetry.addData("intakeArray 1:", intakeIndexColors[1]);
        telemetry.addData("intakeArray 2:", intakeIndexColors[2]);


        telemetry.update();


    }
    @Override
    public void stop(){


    }
}





