/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
Additional modifications by michaudc 2021
Modifications for CENTERSTAGE by michaudc 2023
Modifiations for DECODE by michaudc 2025
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MaristBot2025: Teleop Strafer 2025", group="Training")
//@Disabled
public class TeleopStrafer_Quad_2025 extends OpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2025_Quad robot   = new MaristBaseRobot2025_Quad(); // use the class created to define a Robot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    GoBildaPinpointDriver odo;
    
    // Values based on DcMotorEx and .setVelocity()    
    private double SHOOTER_FULL = -1260;
    private double SHOOTER_HALF = -600;
    private double SHOOTER_VELOCITY = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Initialize the GoBilda Odometry Computer
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(90.0, 0.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        

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
        armPos = 50; // Lifts Arm for Driving
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        
        /*
        Request an update from the Pinpoint odometry computer. This checks almost all outputs
        from the device in a single I2C read.
         */
        odo.update();
        
        // Variables for Joystick Paddle input
        double leftX;
        double leftY;
        double rightX;
        double rightY;

        // Strafer Mode
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x ;
        rightY = gamepad1.right_stick_y;
        
        // For Field Centric
        Pose2D pose = odo.getPosition();
        double heading = pose.getHeading(AngleUnit.DEGREES);
        //robot.driveFieldCentric(leftX, leftY, rightX, heading);
        
        // Reset Odometry Computer Heading and Location
        if (gamepad1.a) {
            odo.resetPosAndIMU();
        }
        
        // For Robot Centric
        robot.driveStrafer(leftX, leftY, rightX);
        
        // Launcher Value Settings based on dpad
        if (gamepad1.dpad_up) {
            SHOOTER_VELOCITY = SHOOTER_FULL;
        }
        if (gamepad1.dpad_left) {
            SHOOTER_VELOCITY += 1;
        }
        if (gamepad1.dpad_right) {
            SHOOTER_VELOCITY -= 1;
        }
        if (gamepad1.dpad_down) {
            SHOOTER_VELOCITY = 0;
        }
        // Range Checking - leftArm should not go backwards
        if (SHOOTER_VELOCITY > 0) {
            SHOOTER_VELOCITY = 0;
        }
        // Set the leftArm to the proper velocity
        robot.leftArm.setVelocity(SHOOTER_VELOCITY);
        
        // Feeder Code - Continous Rotation Servos
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

        // Intake Code
        //TODO: Finish This
        
        // Diagnostic Outputs - Velocity of Launcher Motor (leftArm)
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


