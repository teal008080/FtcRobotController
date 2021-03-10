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
        robot.wobbleSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
        robot.drop.setPosition(.6);

    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */



    @Override
    public void loop() {


        //robot.drop.setPosition(0);
        // telemetry.addData("Front Distance", robot.dSensorFront.getDistance(DistanceUnit.INCH));
        //telemetry.addData("Back Distance", robot.dSensorBack.getDistance(DistanceUnit.INCH));

        //Double Variables for driver control sticks
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double z = -gamepad1.right_stick_x;


        if (gamepad1.left_bumper) { // Control Speed of Drive
            robot.speedFactor = 2;
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
        robot.backleftDrive.setPower((Math.pow((y + x) * robot.reverseFactor - z, 3)) / robot.speedFactor);
        robot.backrightDrive.setPower((Math.pow((-y + x) * robot.reverseFactor - z, 3)) / robot.speedFactor);

        telemetry.addData("LF", robot.frontleftDrive.getPower());
        telemetry.addData("RF", robot.frontrightDrive.getPower());
        telemetry.addData("LB", robot.backleftDrive.getPower());
        telemetry.addData("RB", robot.backrightDrive.getPower());


        //Intake drive


        double intakereverse;

        if(gamepad2.left_bumper) {
            intakereverse = -1;
        }   else {
            intakereverse = 1;
        }
        if (gamepad2.y && !robot.intakeToggle) {
            if (robot.intakeChainDrive.getPower() == 0) robot.intakeChainDrive.setPower(intakereverse);
            else robot.intakeChainDrive.setPower(0);
            robot.intakeToggle = true;
            if (robot.intakeChainDrive.getPower() == 0)
                telemetry.addData("Intake", "Deactivated");
            else telemetry.addData("Intake", "Activated");
        } else if (!gamepad2.y) robot.intakeToggle = false;

        double shotspeed = .7;
        //Shooter code
        if (gamepad1.right_bumper) {
            shotspeed = .63;
        }   else {
            shotspeed = .7;
        }

        if (gamepad2.x && !robot.shooterToggle) {
            if (robot.shooterDrive.getPower() == 0) robot.shooterDrive.setPower(shotspeed);
            else robot.shooterDrive.setPower(0);
            robot.shooterToggle = true;
            if (robot.shooterDrive.getPower() == 0) telemetry.addData("Shooter", "Deactivated");
            else telemetry.addData("Shooter", "Activated");
        } else if (!gamepad2.x) robot.shooterToggle = false;


        // Trigger Mechanism

        if (gamepad1.b) {
            while (gamepad1.b) {
                if (robot.waittime == 0) robot.waittime = System.currentTimeMillis();
                if (robot.triggerServo.getPosition() == .43 && System.currentTimeMillis() - robot.waittime == 140) {
                    robot.triggerServo.setPosition(.55);
                    robot.waittime = 0;
                }

                if (robot.triggerServo.getPosition() == .55 && System.currentTimeMillis() - robot.waittime == 140) {
                    robot.triggerServo.setPosition(.43);
                    robot.waittime = 0;
                }


            }
        }

        if (!gamepad1.b) { robot.triggerServo.setPosition(.43); robot.waittime = 0; telemetry.addData("Trigger","Deactivated"); }

        //wobble meche

        robot.wobbleSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("pos", robot.wobbleSpool.getCurrentPosition());
        telemetry.addData("posinteded", robot.wobbleSpool.getTargetPosition());
        telemetry.addData("pos", robot.wobbleGrab.getPosition());
        if (gamepad2.a && !robot.wobbleDown) {
            if (Math.abs(robot.wobbleSpool.getCurrentPosition() - robot.wobbleSpool.getTargetPosition()) <= robot.wobbletolerance){
                telemetry.addData("pos", robot.wobbleSpool.getCurrentPosition());
                robot.wobbleSpool.setPower(.5);

                robot.wobbleSpool.setTargetPosition(1000);

            }
            else {
                robot.wobbleSpool.setPower(.5);
                robot.wobbleSpool.setTargetPosition(0);
            }
            robot.wobbleDown = true;
            if (Math.abs(robot.wobbleSpool.getCurrentPosition() - robot.wobbleSpool.getTargetPosition()) <= robot.wobbletolerance)
                telemetry.addData("Wobble","Down");
            else telemetry.addData("Wobble", "Up");
        } else if (!gamepad2.a) robot.wobbleDown = false;

        if (gamepad2.b && !robot.wobbleopen) {
            if (robot.wobbleGrab.getPosition() == 0.4){



                robot.wobbleGrab.setPosition(.67);

            }
            else {

                robot.wobbleGrab.setPosition(.4);


            }
            robot.wobbleopen = true;
            if (robot.wobbleGrab.getPosition() == 0.4)
                telemetry.addData("Wobble2","Opne");
            else telemetry.addData("Wobble2", "closed");
        } else if (!gamepad2.b) robot.wobbleopen = false;


            /*
            if (robot.triggerboolean && gamepad1.b) {  // Only execute once per Button push
                robot.triggerboolean = false;
                // Prevents this section of code from being called again until the Button is released and re-pressed
                if (robot.triggerstate) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
                    robot.triggerstate = false;
                    robot.trigger.setPosition(.43);
                    telemetry.addData("Trigger", "Deactivated");
                    robot.triggerboolean = true;
                } else {
                    if (System.currentTimeMillis() - robot.waittime >= 84) {
                        robot.trigger.setPosition(.43);
                        robot.waittime = 0;
                        return;
                    } else {
                        if (robot.waittime == 0) {
                            robot.waittime = System.currentTimeMillis();
                        }
                        robot.trigger.setPosition(.53);
                        telemetry.addData("Trigger", "Activated");
                        robot.triggerstate = true;
                        robot.triggerboolean = true;
                    }
                }
             */



    }



            /*
            if(gamepad1.b){
                robot.trigger.setPosition(.53);
            }
            if(gamepad1.a){
                robot.trigger.setPosition(.43);
            }
*/



    @Override
    public void stop() {
    }

}