/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.configVars.FASTVELOCITYMULT;
import static org.firstinspires.ftc.teamcode.configVars.GREENHIGHTHRESHOLD;
import static org.firstinspires.ftc.teamcode.configVars.GREENLOWTHRESHOLD;
import static org.firstinspires.ftc.teamcode.configVars.LAUNCHSERVOFINAL;
import static org.firstinspires.ftc.teamcode.configVars.LAUNCHSERVOINIT;
import static org.firstinspires.ftc.teamcode.configVars.PURPLEHIGHTHRESHOLD;
import static org.firstinspires.ftc.teamcode.configVars.PURPLELOWTHRESHOLD;
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


@TeleOp(name="Field Centric Mecanum Teleop", group="Linear OpMode")

public class mecanumFieldCentricTeleop extends LinearOpMode {

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

    private Servo RGB;

    int[] intakeIndexColors = {0, 0, 0};

    private NormalizedColorSensor colorSensor = null;


    Limelight3A limelight;


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
        RGB = hardwareMap.get(Servo.class, "led_light");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class,"sensor_color");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        revolverMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        revolverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        int revolverCountsPerRev = 538;
        double fastDriveSpeed = 1;
        double slowDriveSpeed = 0.5;
        double distanceFromAprilTag = 1;
        double maxSpeed = 1;
        double rotY;
        boolean isFieldCentricModeOn = false;
        double rotX;
        double driveSpeedMult = fastDriveSpeed;
        double TPS = 2800;
        boolean isOnBlue = true;
        boolean allianceToggled = false;
        boolean driveModeToggled = false;
        boolean driveSpeedToggled = false;
        boolean servoIntakeToggled = false;
        boolean servoLauncherToggled = false;
        boolean isIntakeServoUp = false;
        boolean isLauncherServoUp = false;
        boolean isRevolverSlowModeOn = false;
        boolean revolverJiggleMode = false;
        double revolverJiggleFrequency = 7;
        double revolverJiggleAmplitude = 4;
        int revolverJiggleOffset = 0;
        double max = 0;
        double launchServoInitialPos = 0;
        double launchServoEndPos = 1;
        double revolverTargetPosition = 0;
        double limelightVelocityMult = 0;
        double x = 0;
        double y = 0;


        ElapsedTime timeSinceStart = new ElapsedTime();
        revolverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        revolverTargetPosition = 0;
        revolverMotor.setTargetPosition((int) revolverTargetPosition);
        revolverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
       // NormalizedRGBA colors= colorSensor.getNormalizedColors();
        colorSensor.setGain(3);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        final float[] hsvValues = new float[3];

        while (opModeIsActive()) {

/*
                LLResult result = limelight.getLatestResult();
                Pose3D botpose = result.getBotpose();
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                if (isOnBlue) {
                    distanceFromAprilTag = Math.sqrt(Math.pow((botpose.getPosition().x + 1.825), 2)
                            + Math.pow((botpose.getPosition().y + 1.825), 2));
                }
                else{
                    distanceFromAprilTag = Math.sqrt(Math.pow((botpose.getPosition().x + 1.825), 2)
                            + Math.pow((botpose.getPosition().y - 1.825), 2));

                }
                limelightVelocityMult = -0.08633489*Math.pow(distanceFromAprilTag, 3) + 0.398921 * Math.pow(distanceFromAprilTag, 2) - 0.250368;
                telemetry.addData("x", x);
                telemetry.addData("y", y);
                telemetry.addData("Tx", tx);
                telemetry.addData("Ty", ty);
                telemetry.addData("Ta", ta);
                telemetry.addData("distance From AprilTag", distanceFromAprilTag);
                telemetry.addData("limelight velocity mult", limelightVelocityMult);

                launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
*/
         //   launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


            if (gamepad1.b) {
                    revolverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    revolverTargetPosition = 0;
                    revolverMotor.setTargetPosition((int) revolverTargetPosition);
                    revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                y = scaleInput(-gamepad1.left_stick_y, 1.5, true);  // Note: pushing stick forward gives negative value
                x = scaleInput(gamepad1.left_stick_x, 1.5, true);
                double rx = scaleInput(gamepad1.right_stick_x, 1.5, true);

           /* if (gamepad1.dpad_right){

                launcherServo.setPosition(1);
            }
*/
            /*
            if (gamepad1.dpad_up){

                intakeServo.setPower(1);
            } else if (gamepad1.dpad_down) {

                intakeServo.setPower(-1);
            }
            else{
                intakeServo.setPower(0);
            }

*/

            if (revolverMotor.isBusy()){

               // RGB.setPosition(0.9);
            }
            else{
                //RGB.setPosition(0.5);
            }

            if (intakeIndexColors[2] == 1){
                RGB.setPosition(0.7);


            }
            else if (intakeIndexColors[2] == 2){
               RGB.setPosition(0.45);
            }

            else{
                RGB.setPosition(0.3);
            }
                NormalizedRGBA colors= colorSensor.getNormalizedColors();
                colorSensor.setGain(3);

               Color.colorToHSV(colors.toColor(), hsvValues);

if (!revolverMotor.isBusy()) {
    if (hsvValues[0] > PURPLELOWTHRESHOLD && hsvValues[0] < PURPLEHIGHTHRESHOLD) {
        telemetry.addData("purple", 2);
        intakeIndexColors[0] = 1;
        //RGB.setPosition(0.7);


    } else if (hsvValues[0] > GREENLOWTHRESHOLD && hsvValues[0] < GREENHIGHTHRESHOLD) {

        telemetry.addData("green", 1);
        intakeIndexColors[0] = 2;
        // RGB.setPosition(0.45);


    } else {

        telemetry.addData("no ball", 0);
        //  RGB.setPosition(0.3);
    }
}
            /*
            if (launcherServo.getPosition() > LAUNCHSERVOINIT){
                revolverMotor.setPower(0);
            }
*//*
            if (revolverJiggleMode) {
                revolverJiggleOffset = (int) (Math.sin(timeSinceStart.seconds() * WIGGLEFREQUENCY) * WIGGLEAMPLITUDE);
                revolverMotor.setTargetPosition(revolverMotor.getTargetPosition() + revolverJiggleOffset);
                revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (revolverJiggleOffset < 0){
                    revolverMotor.setPower(-1);
                } else if (revolverJiggleOffset >= 0){
                    revolverMotor.setPower(1);
                }
            }
*/
                if (gamepad1.dpad_right && gamepad1.a) {
                    revolverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    revolverMotor.setTargetPosition(0);
                    revolverMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    revolverMotor.setPower(0.2);
                    isRevolverSlowModeOn = true;


                } else if (gamepad1.dpad_left && gamepad1.a) {
                    revolverMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    revolverTargetPosition = 0;
                    revolverMotor.setTargetPosition(0);
                    revolverMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    revolverMotor.setPower(-0.2);
                    isRevolverSlowModeOn = true;

                } else if (gamepad1.dpad_right && revolverMotor.isBusy() == false) {
                    revolverTargetPosition += 537.7 / 3;
                    revolverMotor.setTargetPosition((int) revolverTargetPosition);
                    revolverMotor.setPower(1);
                    isRevolverSlowModeOn = false;
                    shiftIntakeArrayLeft();
                    revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                } else if (gamepad1.dpad_left && revolverMotor.isBusy() == false) {
                    revolverTargetPosition -= 537.7 / 3;
                    revolverMotor.setTargetPosition((int) revolverTargetPosition);
                    revolverMotor.setPower(-1);
                    isRevolverSlowModeOn = false;
                    shiftIntakeArrayRight();
                    revolverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                } else if (gamepad1.dpad_up) {
                    revolverMotor.setPower(0);

                } else if (isRevolverSlowModeOn == true) {

                    revolverMotor.setPower(0);

                } else {
                    revolverMotor.setPower((revolverMotor.getTargetPosition() - revolverMotor.getCurrentPosition()));
                }
                //evil evil evil evil scale input code
/*
            if (Math.max(gamepad1.right_trigger, gamepad1.left_trigger) > 0.1){
                launchMotor.setVelocity(Math.max(gamepad1.right_trigger, gamepad1.left_trigger) * TPS * VELOCITYMULT);
                telemetry.addData("Launcher power", launchMotor.getPower());
                telemetry.addData("launcher velocity:", launchMotor.getVelocity());
            }
*/

                //good sane and normal launcher code

                if (gamepad1.left_trigger >= 0.5) {
                    launchMotor.setVelocity(TPS * SLOWVELOCITYMULT);
                } else if (gamepad1.right_trigger >= 0.5) {

                    launchMotor.setVelocity(TPS * FASTVELOCITYMULT);
                } else if (gamepad1.dpad_down){
                    launchMotor.setVelocity(0);
                }

/*
            if (gamepad1.dpad_up){
                revolverJiggleMode = true;
            } else if (gamepad1.dpad_down) {
                revolverJiggleMode = false;
            }
*/
/*
            if (gamepad1.x && isIntakeServoUp == true && !servoIntakeToggled) {
                intakeServo.setPosition(0);
                isIntakeServoUp = false;
                servoIntakeToggled = true;
            }
            else if (gamepad1.x && isIntakeServoUp == false && !servoIntakeToggled){
                intakeServo.setPosition(0.5);
                isIntakeServoUp = true;
                servoIntakeToggled = true;
            }
            else if(!gamepad1.x){

                servoIntakeToggled = false;
            }
*/

                if (revolverMotor.isBusy()){
                    launcherServo.setPosition((LAUNCHSERVOINIT));
            }

                if (gamepad1.x && isLauncherServoUp == true && !servoLauncherToggled) {
                    launcherServo.setPosition(LAUNCHSERVOINIT);
                    isLauncherServoUp = false;
                    servoLauncherToggled = true;
                } else if (gamepad1.x && isLauncherServoUp == false && !servoLauncherToggled) {
                    launcherServo.setPosition(LAUNCHSERVOFINAL);
                    intakeIndexColors[2] = 0;
                    isLauncherServoUp = true;
                    servoLauncherToggled = true;
                } else if (!gamepad1.x) {
                    servoLauncherToggled = false;
                }


         /*       NormalizedRGBA colors= colorSensor.getNormalizedColors();
                colorSensor.setGain(3);
*/
        //    Color.colorToHSV(colors.toColor(), hsvValues);



                //  telemetry.addData("1",slot1);
                //  telemetry.addData("2",slot2);
                //  telemetry.addData("3",slot3);
                //telemetry.addData("h",hsvValues[0]);
                //  telemetry.addData("v",hsvValues[1]);
                //  telemetry.addData("red",colors.red);
                //  telemetry.addData("green",colors.green);
                //   telemetry.addData("blue",colors.blue);


                /*if (hsvValues[0] > 190 && hsvValues[0] < 255) {
                    telemetry.addData("purple", 2);
                    //RGB.setPosition(0.7);


                } else if (hsvValues[0] > 120 && hsvValues[0] < 170) {

                    telemetry.addData("green", 1);
                   // RGB.setPosition(0.45);


                } else {

                    telemetry.addData("no ball", 0);
                  //  RGB.setPosition(0.3);

                }
/**/
                if (gamepad1.back && isOnBlue == true && !allianceToggled) {

                    isOnBlue = false;
                    limelight.pipelineSwitch(2);
                    allianceToggled = true;
                } else if (gamepad1.back && isOnBlue == false && !allianceToggled) {

                    isOnBlue = true;
                    limelight.pipelineSwitch(3);
                    allianceToggled = true;
                } else if (!gamepad1.back) {
                    allianceToggled = false;
                }


                if (gamepad1.left_stick_button && isFieldCentricModeOn == true && !driveModeToggled) {

                    isFieldCentricModeOn = false;
                    driveModeToggled = true;
                } else if (gamepad1.left_stick_button && isFieldCentricModeOn == false && !driveModeToggled) {
                    isFieldCentricModeOn = true;
                    driveModeToggled = true;
                } else if (!gamepad1.left_stick_button) {
                    driveModeToggled = false;
                }

                if (gamepad1.start) {
                    imu.resetYaw();
                }

                //drive speed toggle
                if (gamepad1.y && driveSpeedMult == fastDriveSpeed && !driveSpeedToggled) {

                    driveSpeedMult = slowDriveSpeed;
                    driveSpeedToggled = true;
                } else if (gamepad1.y && driveSpeedMult < fastDriveSpeed && !driveSpeedToggled) {
                    driveSpeedMult = fastDriveSpeed;
                    driveSpeedToggled = true;

                } else if (!gamepad1.y) {
                    driveSpeedToggled = false;
                }

                telemetry.addData("is field centric mode on:", isFieldCentricModeOn);
                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                /*code to get the launcher working -- uncomment later


                 */
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                telemetry.addData("botHeading: ", botHeading);
                telemetry.addData("revolver Motor Target Pos:", revolverMotor.getTargetPosition());
                telemetry.addData("revolver Motor Target Pos Variable:", revolverTargetPosition);
                telemetry.addData("revolver Motor Current Pos:", revolverMotor.getCurrentPosition());
                telemetry.addData("isOnBlue:", isOnBlue);
                // Rotate the movement direction counter to the bot's rotation
                if (isFieldCentricModeOn) {
                    rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading));
                    rotY = (x * Math.sin(-botHeading) + y * Math.cos(-botHeading));
                } else {
                    rotX = x;
                    rotY = y;
                }
                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower = (rotY + rotX + rx) * driveSpeedMult;
                double leftBackPower = (rotY - rotX + rx) * driveSpeedMult;
                double rightFrontPower = (rotY - rotX - rx) * driveSpeedMult;
                double rightBackPower = (rotY + rotX - rx) * driveSpeedMult;
                double intakePower;
                if (gamepad1.left_bumper) {
                    intakePower = 1.0;
                } else if (gamepad1.right_bumper) {
                    intakePower = -1;
                } else if (revolverMotor.isBusy()) {
                    intakePower = 1;
                } else {
                    intakePower = 0.0;
                }


                intakeMotor.setPower(intakePower);
                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
                max = Math.max(max, abs(leftBackPower));
                max = Math.max(max, abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

//             This is test code:
//
//             Uncomment the following code to test your motor directions.
//             Each button should make the corresponding motor run FORWARD.
//               1) First get all the motors to take to correct positions on the robot
//                  by adjusting your Robot Configuration if necessary.
//               2) Then make sure they run in the correct direction by modifying the
//                  the setDirection() calls above.
//             Once the correct motors move in the correct direction re-comment this code.

                //           leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
                //         leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
                //       rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
                //     rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Revolver Power", revolverMotor.getPower());
                telemetry.addData("intakeArray 0:", intakeIndexColors[0]);
            telemetry.addData("intakeArray 1:", intakeIndexColors[1]);
            telemetry.addData("intakeArray 2:", intakeIndexColors[2]);

            telemetry.update();

        }
    }

    private double scaleInput(double input, double power, boolean active) {
        double output = input;
        if (active) {
            if (input < 0) {
                output = input * (Math.pow(-input, power));
            } else {
                output = input * (Math.pow(input, power));
            }
        }
        telemetry.addData("scaled value", output);
        return output;
    }
}