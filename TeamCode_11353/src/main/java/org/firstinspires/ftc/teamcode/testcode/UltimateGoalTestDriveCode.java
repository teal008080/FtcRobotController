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
             * SHOOTER CODE
             */

            //Belt Toggle (GAMEPAD A)
            if (robot.beltToggle && gamepad1.a) {  // Only execute once per Button push
                robot.beltToggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                if (robot.belt) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                    robot.belt= false;
                    robot.conveyerDrive.setPower(0);
                    telemetry.addData("Feed Belt", "Deactivated");
                } else {
                    robot.belt= true;
                    robot.conveyerDrive.setPower(1);
                    telemetry.addData("Feed Belt", "Activated");
                }
            } else if(!gamepad1.a) {
                robot.beltToggle = true; // Button has been released, so this allows a re-press to activate the code above.
            }

            //Shooting Toggle (RIGHT TRIGGER)

            if (gamepad1.right_trigger == 1) {  //Converts the trigger from a float to a boolean
                robot.shootTrigger = true;
            } else if (gamepad1.right_trigger == 0) {
                robot.shootTrigger = false;
            }

            if (robot.shootToggle && robot.shootTrigger) {  // Only execute once per Button push
                robot.shootToggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                if (robot.shoot) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                    robot.shoot= false;
                    robot.shooterDrive.setPower(0);
                    telemetry.addData("Shooter", "Deactivated");
                } else {
                    robot.shoot= true;
                    robot.shooterDrive.setPower(1);
                    telemetry.addData("Shooter", "Activated");
                }
            } else if(!robot.shootTrigger) {
                robot.shootToggle = true; // Button has been released, so this allows a re-press to activate the code above.
            }



        }
        @Override
        public void stop() {
        }

    }

