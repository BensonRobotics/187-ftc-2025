package org.firstinspires.ftc.teamcode.PedroPathing;

import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@Autonomous(name = "League Meet 0 Auto")

public class lm0Auto extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeMotor = null;


    public void runOpMode(){



        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        //private final Pose startPose = new Pose(28.5, 128, Math.toRadians(90)); // Start Pose of our robot.
        //private final Pose scorePose1 = new Pose(35.37, 119.3, Math.toRadians(145)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        //private final Pose leaveLaunchLine = new Pose(53, 57.8, Math.toRadians(145));

   //     FuturePose scorePose;
     //   scorePose;
       // PathChain pathChain = follower.pathBuilder()
                //path to go to the location for the first launch
         //       .addPath(new BezierLine(startPose, scorePose1))
           //     .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
             //   .addPath(new BezierLine(pickup1Pose, scorePose))
               // .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                //.build();


    }

}
