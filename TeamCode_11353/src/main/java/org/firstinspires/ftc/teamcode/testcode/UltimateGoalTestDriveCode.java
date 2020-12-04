package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is the Test Driver Control for the 2020 LCL Lightning team 11353 Robot (unnamed)
 */

    @TeleOp(name="UltimateGoalTestDriveCode", group="Iterative Opmode")//Originally just DriverControl
//@Disabled
    public class UltimateGoalTestDriveCode extends OpMode
    {
        private ElapsedTime runtime = new ElapsedTime();


        //Creates new robot
        UltimateGoalTestHardwareMap robot       = new UltimateGoalTestHardwareMap();//probably want to update class + function names

        /**
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            telemetry.addData("Status", "Initialized");

            //Initalize hardware from Hardware UltimateGoal
            robot.init(hardwareMap);


            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

        /**
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {


            /**
             * Code to run ONCE when the driver hits PLAY
             */
        }
        @Override
        public void start() {}

        public boolean changed1, on1 = false;



        @Override
        public void loop() {

            /**
             * Intake Code
              */

            //Intake Toggle

            if (robot.intakeToggle && gamepad1.y) {  // Only execute once per Button push
                robot.intakeToggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                if (robot.intake) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                    robot.intake = false;
                    robot.intakeDrive.setPower(0);
                    telemetry.addData("Intake", "Deactivated");
                } else {
                    robot.intake = true;
                    robot.intakeDrive.setPower(1);
                    telemetry.addData("Intake", "Activated");
                }
            }









        }
        @Override
        public void stop() {
        }

    }

