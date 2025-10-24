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

@TeleOp(name="MaristBot2025: Teleop Strafer 2025", group="Training")
//@Disabled
public class TeleopStrafer_Quad_2025 extends OpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2025_Quad robot   = new MaristBaseRobot2025_Quad(); // use the class created to define a Robot's hardware

    private double SPEED_CONTROL = 0.5;
    
    private double SHOOTER_FULL = -1600;
    private double SHOOTER_HALF = -1500;
    private double SHOOTER_VELOCITY = 0;
    
    private double NEW_P = 160;
    private double NEW_I = 1.5;
    private double NEW_D = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set PID for Launcher
        PIDCoefficients pidSettings = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        robot.leftArm.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);

        // Wait for driver to press start
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
        
        // Driving Code
        double leftX = gamepad1.left_stick_x * SPEED_CONTROL;
        double leftY = gamepad1.left_stick_y * SPEED_CONTROL;
        double rightX = = gamepad1.right_stick_x * SPEED_CONTROL;

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
        else if (gamepad1.right_triger > 0.5) {
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

}


