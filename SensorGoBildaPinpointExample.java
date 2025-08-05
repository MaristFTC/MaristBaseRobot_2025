/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */
 
 /*
 
 Code added to incorperate a drive to point to simulate Autonomous Motion
 This is a Test class for proof of concept. Mr. Michaud August 2025
 
 */

@TeleOp(name="goBILDA Pinpoint Example", group="Linear OpMode")
//@Disabled

public class SensorGoBildaPinpointExample extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;
    
    private MaristBaseRobot2025_Quad robot = new MaristBaseRobot2025_Quad();
    
    // fields for Drive to Point
    // These values on are in millimeters for now
    // TODO: Need to update to Inches
    final double DESIRED_DISTANCE = 10.0; // MM
    
    // Forward Speed control
    final double SPEED_GAIN = 0.01;
    
    // Strafe Gain
    final double STRAFE_GAIN = 0.015;
    
    // Turn Gain
    final double TURN_GAIN = 0.05;
    
    // Max Speeds
    final double MAX_AUTO_SPEED = 0.75;
    final double MAX_AUTO_STRAFE = 0.75;
    final double MAX_AUTO_TURN = 0.5;
    
    // Test Target Position
    // Distances in MM
    // rotation in degrees
    double xTarget = 254;
    double yTarget = -100;
    double thetaTarget = 90;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        robot.init(hardwareMap);

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-180.0, 30.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();
        
        // Auto Driving variables and setup
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);


            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }
            
            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            // Get Current Position
            double xPos = pos.getX(DistanceUnit.MM);
            double yPos = pos.getY(DistanceUnit.MM);
            double thetaPos = pos.getHeading(AngleUnit.DEGREES);
            
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", xPos, yPos, thetaPos);
            telemetry.addData("Position", data);
            
            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();
            
            // Compute Errors Process:
            
                
            // Compute Errors
            double rangeError = getRangeError(xPos, yPos, xTarget, yTarget);
            double headingError = Math.toDegrees(Math.atan2((yPos-yTarget), (xPos-xTarget)));
            double yawError = thetaTarget - thetaPos;
            
            String data2 = String.format(Locale.US, "{Distance Error: %.3f, Heading Error: %.3f, Yaw Error: %.3f}", rangeError, headingError, yawError);
            telemetry.addData("Errors", data2);
            
            String data3 = String.format(Locale.US, "{Drive Power: %.3f, Strafe Power: %.3f, Turn Power: %.3f}", drive, strafe, turn);
            telemetry.addData("Errors", data3);
            
            
            // If left bumper pressed = drive to Target
            // Else - manual driving
            if (gamepad1.left_bumper) {
                
                //drive = 0;
                //turn = 0;
                //strafe = 0;
                
                // Adjust with gain
                drive = Range.clip((xPos-xTarget) * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(-yawError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip((yPos-yTarget) * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                
            }
            else {
                /*
                Get values from remote and drive in Straffer Mode
                */
                strafe = gamepad1.left_stick_x;
                drive = gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;
            
                
            }
            
            //robot.driveStrafer(strafe, drive, turn);
            robot.driveFieldCentric(strafe, drive, turn, thetaPos);
            
            
            

            

        }
    }
    
    private double getRangeError(double x1, double y1, double x2, double y2) {
        double distance = 0;
        
        double diff = Math.pow((x1-x2), 2) + Math.pow((y1-y2), 2);
        distance = Math.sqrt(diff);
        
        return distance;
    }
    
}

