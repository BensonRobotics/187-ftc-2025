package org.firstinspires.ftc.teamcode.PedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous(name = "PedroPathing 3 Motif Auto")

public class threeMotifAuto extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeMotor = null;

    private DcMotorEx launchMotor = null;

    private final Pose startPose = new Pose(34.32558139534884, 134.1627906976744, Math.toRadians(90));
    private final Pose detectMotifPose = new Pose(63.418604651162795, 115.32558139534883, Math.toRadians(90));
    private final Pose launchMotifPose = new Pose(41.23255813953489, 121.3953488372093, Math.toRadians(145));
    private final Pose MotifOnePose = new Pose(41.86046511627907, 83.30232558139534, Math.toRadians(0));
    private final Pose IntakeMotifOnePose = new Pose(28.465116279069765, 83.30232558139534, Math.toRadians(0));
    private final Pose MotifTwoPose = new Pose(41.86046511627907, 60.06976744186046, Math.toRadians(0));
    private final Pose IntakeMotifTwoPose = new Pose(29.302325581395348, 60.06976744186046, Math.toRadians(0));
    private int pathState;

    /*
        motif 1 is ppg
        motif 2 is pgp
        motif 3 is gpp
     */
    private int motif = 0;
    private Path scorePreload;
    private PathChain launchMotifPreload, prepareMotifOneIntake, intakeMotifOne, launchMotifOne, prepareMotifTwoIntake, intakeMotifTwo, launchMotifTwo;


    private void setPathState(int i) {
        pathState = i;
    }

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, detectMotifPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), detectMotifPose.getHeading());

        //moves to launch the first motif
        launchMotifPreload = follower.pathBuilder()
                .addPath(new BezierLine(detectMotifPose, launchMotifPose))
                .setLinearHeadingInterpolation(detectMotifPose.getHeading(), launchMotifPose.getHeading())
                .build();

        //moves to intake the first motif
        prepareMotifOneIntake = follower.pathBuilder()
                .addPath(new BezierLine(launchMotifPose, MotifOnePose))
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
                .addPath(new BezierLine(launchMotifPose, MotifTwoPose))
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

    public void detectMotifMethod() {

        //enter in code to detect the motif

    }

    public void launchMotif(){

        //launches the motif
    }

    public void intakeMotif(){

        //set intake to run

    }

    public void autonomousPathUpdate() {


            switch(pathState){
                case 0:
                    follower.followPath(scorePreload);
                    setPathState(1);
                    break;

                //detects motif
                case 1:
                    detectMotifMethod();
                    follower.followPath(launchMotifPreload);
                    launchMotif();

                    setPathState(2);

                case 2:
                    follower.followPath(prepareMotifOneIntake);
                    setPathState(3);

                case 3:
                    intakeMotif();
                    follower.followPath(intakeMotifOne);
                    setPathState(4);

                case 4:
                    follower.followPath(launchMotifOne);
                    launchMotif();
                    setPathState(5);

                case 5:
                    follower.followPath(launchMotifOne);
                    setPathState(6);
                case 6:
                    follower.followPath(prepareMotifTwoIntake);
                    setPathState(7);

                case 7:
                    intakeMotif();
                    follower.followPath(intakeMotifTwo);
                    setPathState(8);

                case 8:
                    follower.followPath(launchMotifTwo);
                    launchMotif();




            }
        }

    public void runOpMode(){



        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        //private final Pose startPose = new Pose(28.5, 128, Math.toRadians(90)); // Start Pose of our robot.
        //private final Pose scorePose1 = new Pose(35.37, 119.3, Math.toRadians(145)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        //private final Pose leaveLaunchLine = new Pose(53, 57.8, Math.toRadians(145));



    }
}


    }
