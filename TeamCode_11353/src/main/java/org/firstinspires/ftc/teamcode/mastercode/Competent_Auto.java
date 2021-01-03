// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode.mastercode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.robotutils.MathFunctions;
import org.firstinspires.ftc.teamcode.robotutils.RobotMovement;


@Autonomous(name="Actually Competent Auto", group="PID")
//@Disabled
public class Competent_Auto extends LinearOpMode {

    UltimategoalHardware robot = new UltimategoalHardware();


    @Override
    public void runOpMode() {

        //Initialize servos to starting positions
        robot.init(hardwareMap);

        waitForStart();
        MathFunctions.setAngle();

        //Code above here should never change

        //While the stop button isn't pressed, run this code
        while (!isStopRequested()) {




            //Park
            RobotMovement.drivePIDtime(.5, 90, -1, 250, 0);
            RobotMovement.strafeLeft(1, 1250);

            // Dont put code below here
            RobotMovement.stopDrive();
            break;
        }
        RobotMovement.stopDrive();
        stop();
    }
}
