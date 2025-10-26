/*
Copyright 2025 Marist Robotics by michaudc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;


@Autonomous(name="Auto_Vision_Far_Red_Kitbot_2025", group="Vision")
public class Auto_Vision_Far_Red_KitBot_2025 extends LinearOpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2025_Quad robot   = new MaristBaseRobot2025_Quad();   
    private ElapsedTime runtime = new ElapsedTime();

    // Fields for Camera and PID Control
    private double SHOOTER_FULL = -1560;
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


    @Override
    public void runOpMode() {

        // Setup Robot, Camera, and PID Control
        robot.init(hardwareMap);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set PID Values for Launcher
        PIDCoefficients pidSettings = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        robot.leftArm.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        
        // April Tag Self Driving Fields
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();
        
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // Call Code for Autonomous: Sample Method Below
        driveOffLine();

        
    }
    
    //--- Autonomous Methods ---//

    // Forward and Shoot Auto
    public void driveOffLine() {
        // Start Launch Motor
        robot.leftArm.setVelocity(SHOOTER_HALF);
        
        // Drive Off the Line
        robot.move(36, 0.5);

        // Turn Right
        robot.turn(45, 0.5);

        // Aim
        autoAim(5);

        // Launch
        robot.rightArm.setPower(0.8);
        delay(5);

        // Stop All Motors
        robot.leftArm.setVelocity(0);
        robot.rightArm.setPower(0);

    }

    //--- Utility Methods ---//

    // Sample Delay Code
    // t is in seconds
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }
    
    // Go to April Tag Position
    public void autoAim(double timeout) {
        
        // Set to Run without Encoder for Camera Drive
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        double leftX = 0;
        double leftY = 0;
        double rightX = 0;

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeout)){
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
            
            boolean targetFound     = false; 
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
            if (targetFound) {
                telemetry.addData("Tag No: ", desiredTag.id);
                telemetry.update();

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                double  headingError    = desiredTag.ftcPose.bearing - DESIRED_HEADING;
                double  yawError        = desiredTag.ftcPose.yaw - DESIRED_YAW;
    
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                leftY  = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                rightX   = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                leftX = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                
                // Drive to Target
                robot.driveStrafer(leftX, leftY, rightX);
            }
            else {
                robot.driveStrafer(0, 0, 0); // Stop
            }

        }
        
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
