package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.configVars.FASTVELOCITYMULT;
import static org.firstinspires.ftc.teamcode.configVars.LAUNCHSERVOFINAL;
import static org.firstinspires.ftc.teamcode.configVars.LAUNCHSERVOINIT;
import static org.firstinspires.ftc.teamcode.configVars.SLOWVELOCITYMULT;
import static org.firstinspires.ftc.teamcode.configVars.VELOCITYMULT;
import static org.firstinspires.ftc.teamcode.configVars.WIGGLEAMPLITUDE;
import static org.firstinspires.ftc.teamcode.configVars.WIGGLEFREQUENCY;

import static java.lang.Math.abs;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configVars;

import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name="league meet 3 auto", group="Linear OpMode")

public class lm3Auto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotorEx launchMotor = null;

    private DcMotorEx revolverMotor = null;
    private CRServo intakeServo;
    private Servo launcherServo;

    private NormalizedColorSensor colorSensor = null;


    Limelight3A limelight;

    @Override
    public void runOpMode() {
        IMU imu;
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        /*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(3);
*/

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        revolverMotor = hardwareMap.get(DcMotorEx.class, "revolver_motor");
        // intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
        launcherServo = hardwareMap.get(Servo.class, "launcher_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launchMotor.setMode((DcMotor.RunMode.RUN_USING_ENCODER));
*/
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        revolverMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double TPS = 2800;
        int revolverCountsPerRev = 538;
        double revolverTargetPosition = 0;

        revolverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolverTargetPosition = 0;
        revolverMotor.setTargetPosition((int) revolverTargetPosition);
        revolverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        launcherServo.setPosition(LAUNCHSERVOINIT);
        waitForStart();
        runtime.reset();

            launchMotor.setVelocity(TPS * SLOWVELOCITYMULT);

                leftFrontDrive.setPower(-1);
                leftBackDrive.setPower(-1);
                rightBackDrive.setPower(-1);
                rightFrontDrive.setPower(-1);
                sleep(700);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        telemetry.addData("left_front_target", leftFrontDrive.getTargetPosition());
                telemetry.addData("left_back_target", leftBackDrive.getTargetPosition());
                telemetry.addData("rightFront_target", rightFrontDrive.getTargetPosition());
                telemetry.addData("rightBack_target", rightBackDrive.getTargetPosition());
                telemetry.addData("left_front_current", leftFrontDrive.getCurrentPosition());
                telemetry.addData("left_back_current", leftBackDrive.getCurrentPosition());
                telemetry.addData("rightFront_current", rightFrontDrive.getCurrentPosition());
                telemetry.addData("rightBack_current", rightBackDrive.getCurrentPosition());
                telemetry.update();


        intakeMotor.setPower(1);


            sleep(2000);

            launcherServo.setPosition(LAUNCHSERVOFINAL);
            sleep(1000);
        launcherServo.setPosition(LAUNCHSERVOINIT);
        sleep(1000);

        revolverTargetPosition += 537.7 / 3;
        revolverMotor.setTargetPosition((int) revolverTargetPosition);
        revolverMotor.setPower(1);
        while (revolverMotor.isBusy()){

            revolverMotor.setPower(1);

        }
        sleep(1000);

        launcherServo.setPosition(LAUNCHSERVOFINAL);
        sleep(1000);
        launcherServo.setPosition(LAUNCHSERVOINIT);
        sleep(1000);



        revolverTargetPosition += 537.7 / 3;
        revolverMotor.setTargetPosition((int) revolverTargetPosition);
        revolverMotor.setPower(1);
        while (revolverMotor.isBusy()){

            revolverMotor.setPower(1);

        }
        sleep(1000);

        launcherServo.setPosition(LAUNCHSERVOFINAL);
        sleep(1000);
        launcherServo.setPosition(LAUNCHSERVOINIT);
        sleep(1000);

        while (revolverMotor.isBusy()){

            revolverMotor.setPower(1);

        }

    }
}