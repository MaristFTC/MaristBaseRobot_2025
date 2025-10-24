/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
Additional modifications by michaudc 2021
Modifications for CENTERSTAGE by michaudc 2023
Modifications for DECODE by michaudc 2025
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This file provides basic Telop driving for a Quad Robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the MaristBaseRobot hardware class to define the devices on the robot.
 * All device access is managed through the MaristBaseRobot class.
 * 
 * Additional Methods added for Use of Webcam and April Tags for DECODE 2025 by michaudc
 * 
 */

@TeleOp(name="MaristBot2025: Teleop Strafer Vision 2025", group="Training")
//@Disabled
public class TeleopStrafer_Vision_Quad_2025 extends OpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2025_Quad robot   = new MaristBaseRobot2025_Quad(); // use the class created to define a Robot's hardware

    private double SPEED_CONTROL = 0.5;
        
    private double SHOOTER_FULL = -1600;
    private double SHOOTER_HALF = -1500;
    private double SHOOTER_VELOCITY = 0;
    
    private double NEW_P = 160;
    private double NEW_I = 1.5;
    private double NEW_D = 0;
    
    // for Camera
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 45.0; //  this is how close the camera should get to the target (inches)
    final double DESIRED_HEADING = 5; 
    final double DESIRED_YAW = 0;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set PID for Launcher
        PIDCoefficients pidSettings = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        robot.leftArm.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        
        double leftX;
        double leftY;
        double rightX;
        double rightY;
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected

        // Strafer Mode
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x ;
        rightY = gamepad1.right_stick_y;
        
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
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

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = desiredTag.ftcPose.range - DESIRED_DISTANCE;
            double  headingError    = desiredTag.ftcPose.bearing - DESIRED_HEADING;
            double  yawError        = desiredTag.ftcPose.yaw - DESIRED_YAW;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            leftY  = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            rightX   = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            leftX = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            //telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
            leftX = gamepad1.left_stick_x * SPEED_CONTROL;
            leftY = gamepad1.left_stick_y * SPEED_CONTROL;
            rightX = gamepad1.right_stick_x * SPEED_CONTROL;
        }
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        robot.driveStrafer(leftX, leftY, rightX);
        
        // Shooter
        if (gamepad1.dpad_up) {
            SHOOTER_VELOCITY = SHOOTER_FULL;
        }
        if (gamepad1.dpad_left) {
            SHOOTER_VELOCITY = SHOOTER_HALF;
        }
        if (gamepad1.dpad_down) {
            SHOOTER_VELOCITY = 0;
        }
        
        robot.leftArm.setVelocity(SHOOTER_VELOCITY);
        
        // Feeder with DC Motor
        if (gamepad1.left_trigger > 0.5) {
            //robot.indexMotor.setPower(0.5);
        }
        else if (gamepad1.right_trigger > 0.5) {
            //robot.indexMotor.setPower(-0.5);
        }
        else {
            //robot.indexMotor.setPower(0);
        }

        // Feeder with Servos
        if (gamepad1.left_trigger > 0.5) {
            robot.leftHand.setPosition(1);
            robot.rightHand.setPosition(0);
        }
        else if (gamepad1.right_trigger > 0.5) {
            robot.leftHand.setPosition(0);
            robot.rightHand.setPosition(1);
        }
        else {
            robot.leftHand.setPosition(0.5);
            robot.rightHand.setPosition(0.5);
        }
        
        // Intake with DC Motor
        if (gamepad1.a) {
            robot.rightArm.setPower(0);
        }
        if (gamepad1.b) {
            robot.rightArm.setPower(0.7);
        }
        if (gamepad1.x) {
            robot.rightArm.setPower(-0.7);
        }
        
        // Get PID Information
        PIDCoefficients pidValues = robot.leftArm.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PID Values", "%.04f, %.04f, %.04f", pidValues.p, pidValues.i, pidValues.d);
        
        // Display Launcher Motor Velocity
        double velocity = robot.leftArm.getVelocity();
        telemetry.addData("Velocity", velocity);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    /**
     * Initialize the AprilTag processor.
     */
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
    
}


