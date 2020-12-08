package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This is the Test Driver Control for the 2020 LCL Lightning team 11353 Robot (unnamed)
 */

    @TeleOp(name="UltimateGoalTestDriveCode", group="Iterative Opmode")
//@Disabled
    public class UltimateGoalTestDriveCode extends OpMode
    {
        private ElapsedTime runtime = new ElapsedTime();


        //Creates new robot
        UltimateGoalTestHardwareMap robot       = new UltimateGoalTestHardwareMap();

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


            Orientation imu_angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //Orientation imu2_angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double z_angle = imu_angles.firstAngle;

            //Double Variables for driver control sticks
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double z = -gamepad1.right_stick_x;

            /**
             NEWLY COMMENTED 1/10/20
             Math for robot orientated drive. The Z axis offset is converted to radians. Then by multiplying the y and x values by the
             cos and sin of the gyro, we can "rotate" the gamepad left stick, so forward on the sick is always away
             **/


        double rad = Math.toRadians(z_angle);
        double forward = (y*Math.cos(rad))+(x*Math.sin(rad));
        double side    = (-y*Math.sin(rad))+(x*Math.cos(rad));


        //Assigning drive power to motors using Z-offset
        robot.frontleftDrive.setPower((-forward-side-z)/(robot.speedFactor*robot.reverseFactor));
        robot.frontrightDrive.setPower((-forward+side-z)/(robot.speedFactor*robot.reverseFactor));
        robot.backleftDrive.setPower((forward-side-z)/(robot.speedFactor*robot.reverseFactor));
        robot.backrightDrive.setPower((forward+side-z)/(robot.speedFactor*robot.reverseFactor));




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


            /* OLD DRIVE FUNCTION
            //DRIVE FUNCTION BELOW
            robot.frontleftDrive.setPower((Math.pow((y + x) * robot.reverseFactor + z, 3)) / robot.speedFactor);
            robot.frontrightDrive.setPower((Math.pow((-y + x) * robot.reverseFactor + z, 3)) / robot.speedFactor);
            robot.backleftDrive.setPower((Math.pow((y - x) * robot.reverseFactor + z, 3)) / robot.speedFactor);
            robot.backrightDrive.setPower((Math.pow((-y - x) * robot.reverseFactor + z, 3)) / robot.speedFactor);
            */







        }
        @Override
        public void stop() {
        }

    }

