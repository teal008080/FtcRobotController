package org.firstinspires.ftc.test_teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.test_teamcode.UltimateGoalTestHardwareMap;


/**
     * This is the Main Driver Control for the 2020 FTC Skystone Robot Riccochet
     */

    @TeleOp(name="UltimateGoalTestDriveCode", group="Iterative Opmode")//Originally just DriverControl
//@Disabled
    public class UltimateGoalTestDriveCode extends OpMode
    {
        private ElapsedTime runtime = new ElapsedTime();


        //Creates new robot
        UltimateGoalTestHardwareMap robot       = new UltimateGoalTestHardwareMap();//probably want to update class + function names

        /*
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

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {


            /*
             * Code to run ONCE when the driver hits PLAY
             */
        }
        @Override
        public void start() {}

        public boolean changed1, on1 = false;



        @Override
        public void loop() {






            //SHOOTER CODE


            /*if(gamepad1.a && !changed1) {
              robot.conveyerDrive.setPower(gamepad1.a ? 1 : 0);
              changed1 = true;
            } else if(!gamepad1.a)  {

                changed1 = false;
            }
            */

/*
            boolean state = false;
            int pow = 0;
             if(gamepad1.a){
               state = true;
               pow = 1;
               robot.conveyerDrive.setPower(pow);
                 try {
                     Thread.sleep(1000);
                 } catch (InterruptedException e) {
                     e.printStackTrace();
                 }
             };
             if(state && gamepad1.a){
                 state = false;
                 pow = 0;
                 robot.conveyerDrive.setPower(pow);
                 try {
                     Thread.sleep(1000);
                 } catch (InterruptedException e) {
                     e.printStackTrace();
                 }
             };
             */
            boolean toggle = true;
            boolean belt = false;

            if (toggle && gamepad1.a) {  // Only execute once per Button push
                toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                if (belt) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                    belt= false;
                    conveyerDrive.setPow(0);
                    telemetry.addData("Feed Belt", "Deactivated");
                } else {
                    belt= true;
                    conveyorDrive.setPow(1);
                    telemetry.addData("Feed Belt", "Activated");
                }
            } else if(!gamepad1.a) {
                toggle = true; // Button has been released, so this allows a re-press to activate the code above.
            }


            //INTAKE CODE

           /* boolean toggleb = true;
            boolean intake = false;

            if (toggleb && gamepad1.x) {  // Only execute once per Button push
                toggleb = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
                if (intake) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                    intake= false;
                    intakeDrive.setPow(0);
                    telemetry.addData("Intake", "Deactivated");
                } else {
                    intake= true;
                    intakeDrive.setPow(.45);
                    telemetry.addData("Intake", "Activated");
                }
            } else if(!gamepad1.x) {
                toggleb = true; // Button has been released, so this allows a re-press to activate the code above.
            }
*/


        }
        @Override
        public void stop() {
        }

    }

