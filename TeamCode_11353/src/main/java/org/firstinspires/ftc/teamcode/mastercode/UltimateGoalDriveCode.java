package org.firstinspires.ftc.teamcode.mastercode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
            robot.wobbleSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("pos",robot.wobbleSpool.getCurrentPosition());
           telemetry.addData("data", "test");
            robot.wobbleSpool.setPower(.2);
            if(gamepad1.a){
                robot.wobbleSpool.setTargetPosition(1140);
            } else if(gamepad1.b){
                robot.wobbleSpool.setTargetPosition(0);

            }   else {
                robot.wobbleSpool.setPower(.2);
            }

        }

        @Override
        public void stop() {
        }

    }

