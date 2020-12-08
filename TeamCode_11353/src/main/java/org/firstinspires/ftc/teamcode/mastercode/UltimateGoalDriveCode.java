package org.firstinspires.ftc.teamcode.mastercode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;





    /**
     * This is the Main Driver Control for the 2020 LCL Lightning team 11353 Robot (unnamed)
     */

    @TeleOp(name="UltimateGoalDriveCode", group="Iterative Opmode")

    public class UltimateGoalDriveCode extends OpMode
    {
        private ElapsedTime runtime = new ElapsedTime();


        //Creates new robot
        UltimategoalHardware robot       = new UltimategoalHardware();

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
        public void start() {


        }

        /**
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */



        @Override
        public void loop() {

            /**
             * Gets z Values - Right-Handed Coordinate System
             */



            //Double Variables for driver control sticks
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double z = -gamepad1.right_stick_x;



            if (gamepad1.left_bumper) { // Control Speed of Drive
                robot.speedFactor = 3;
            } else {
                robot.speedFactor = 1;
            }

            if (gamepad1.right_bumper) { //Reverse drive control
                robot.reverseFactor = -1;
            } else {
                robot.reverseFactor = 1;
            }

            //DRIVE FUNCTION BELOW
            robot.frontleftDrive.setPower((Math.pow((y + x) * robot.reverseFactor + z, 3)) / robot.speedFactor);
            robot.frontrightDrive.setPower((Math.pow((-y + x) * robot.reverseFactor + z, 3)) / robot.speedFactor);
            robot.backleftDrive.setPower((Math.pow((y - x) * robot.reverseFactor + z, 3)) / robot.speedFactor);
            robot.backrightDrive.setPower((Math.pow((-y - x) * robot.reverseFactor + z, 3)) / robot.speedFactor);








        }
        @Override
        public void stop() {
        }

    }

