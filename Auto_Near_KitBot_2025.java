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


@Autonomous(name="Auto_Near_Kitbot_2025", group="No Vision")
public class Auto_Near_KitBot_2025 extends LinearOpMode {

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

    @Override
    public void runOpMode() {

        // Setup Robot, Camera, and PID Control
        robot.init(hardwareMap);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set PID Values for Launcher
        PIDCoefficients pidSettings = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        robot.leftArm.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // Call Code for Autonomous: Sample Method Below
        backupAndShoot();

        
    }
    
    //--- Autonomous Methods ---//

    // Backup and Shoot Auto
    public void backupAndShoot() {
        // Start Launch Motor
        robot.leftArm.setVelocity(SHOOTER_HALF);
        
        // backup
        robot.move(-36, 0.5);
                
        // Shoot and intake - uncomment if using DC Motor for Indexer
        //robot.rightArm.setPower(-0.8);

        // Servo Indexer
        robot.leftHand.setPosition(0);
        robot.rightHand.setPosition(1);
        delay(8);
        
        // Stop Shooter and Indexer
        robot.leftArm.setVelocity(0);
        //robot.rightArm.setPower(0);   // DC Motor Indexer Off
        robot.leftHand.setPosition(0.5); // Servos Off
        robot.rightHand.setPosition(0.5); // Servos Off
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

}
