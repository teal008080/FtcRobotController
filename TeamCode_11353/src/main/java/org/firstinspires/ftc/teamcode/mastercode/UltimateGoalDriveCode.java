package org.firstinspires.ftc.teamcode.mastercode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This is the Main Driver Control for the 2020 LCL Lightning team 11353 Robot (unnamed)
 */

@TeleOp(name="DRIVECODE", group="Iterative Opmode")

public class UltimateGoalDriveCode extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();


    //Creates new robot
    UltGoalHwmap2 robot       = new UltGoalHwmap2();

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

        //Double Variables for driver control sticks
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double z = -gamepad1.right_stick_x;


        if (gamepad1.left_bumper) { // Control Speed of Drive
            robot.speedFactor = 3;
        } else {
            robot.speedFactor = 1;
        }

        //DRIVE FUNCTION BELOW
        robot.frontleftDrive.setPower((Math.pow((y + x) + z, 3)) / robot.speedFactor);
        robot.frontrightDrive.setPower((Math.pow((-y + x) + z, 3)) / robot.speedFactor);
        robot.backleftDrive.setPower((Math.pow((y + x) - z, 3)) / robot.speedFactor);
        robot.backrightDrive.setPower((Math.pow((-y + x) - z, 3)) / robot.speedFactor);

        telemetry.addData("LF", robot.frontleftDrive.getPower());
        telemetry.addData("RF", robot.frontrightDrive.getPower());
        telemetry.addData("LB", robot.backleftDrive.getPower());
        telemetry.addData("RB", robot.backrightDrive.getPower());

        //Intake drive

        double intakereverse;

        if (gamepad2.left_bumper) {
            intakereverse = -1;
        } else {
            intakereverse = 1;
        }
        if (gamepad2.y && !robot.intakeToggle) {
            if (robot.intakeChainDrive.getPower() == 0)
                robot.intakeChainDrive.setPower(intakereverse);
            else robot.intakeChainDrive.setPower(0);
            robot.intakeToggle = true;
            if (robot.intakeChainDrive.getPower() == 0)
                telemetry.addData("Intake", "Deactivated");
            else telemetry.addData("Intake", "Activated");
        } else if (!gamepad2.y) robot.intakeToggle = false;

        double shotspeed;
        //Shooter code
        if (gamepad1.right_bumper) {
            shotspeed = 26 * robot.clickMult;
        } else {
            shotspeed = 29 * robot.clickMult;
        }

        if (gamepad2.x && !robot.shooterToggle) {
            if (robot.shooterDrive.getVelocity() == 0) robot.shooterDrive.setVelocity(shotspeed);
            else robot.shooterDrive.setVelocity(0);
            robot.shooterToggle = true;
            if (robot.shooterDrive.getVelocity() == 0) telemetry.addData("Shooter", "Deactivated");
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

        if (!gamepad1.b) {
            robot.triggerServo.setPosition(.43);
            robot.waittime = 0;
            telemetry.addData("Trigger", "Deactivated");
        }

        //wobble meche


            if (gamepad2.a && !robot.wobbletoggle) {
                if (robot.wobbleSpool.getCurrentPosition() <= 5) {
                    robot.wobbleSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.wobbleSpool.setTargetPosition(1000);
                    robot.wobbleSpool.setPower(100);

                } else {
                    robot.wobbleSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.wobbleSpool.setTargetPosition(0);
                    robot.wobbleSpool.setPower(100);
                }
                robot.wobbletoggle = true;
                if (robot.wobbleSpool.getCurrentPosition() == 0) telemetry.addData("Wobble", "Up");
                else telemetry.addData("Wobble", "Down");
            } else if (!gamepad2.a) robot.wobbletoggle = false;

            telemetry.addData("Wobble Pos", robot.wobbleSpool.getCurrentPosition());
            telemetry.addData("Wobble Pow", robot.wobbleSpool.getPower());

            telemetry.addData("Wobble Target Pos", robot.wobbleSpool.getTargetPosition());
            telemetry.addData("Wobble busy?", robot.wobbleSpool.isBusy());

        if (gamepad2.b && !robot.wobbleopentoggle) {
            if (robot.wobbleGrab.getPosition() == 0.4) {
                robot.wobbleGrab.setPosition(.67);
            } else {

                robot.wobbleGrab.setPosition(.4);
            }
            robot.wobbleopentoggle = true;
            if (robot.wobbleGrab.getPosition() == 0.4)
                telemetry.addData("Wobble2", "Opne");
            else telemetry.addData("Wobble2", "closed");
        } else if (!gamepad2.b) robot.wobbleopentoggle = false;





        /*

        telemetry.addData("pos", robot.wobbleSpool.getCurrentPosition());
        telemetry.addData("posinteded", robot.wobbleSpool.getTargetPosition());
        telemetry.addData("pos", robot.wobbleGrab.getPosition());

        if (gamepad2.a && !robot.wobbleDown) {
            if (Math.abs(robot.wobbleSpool.getCurrentPosition() - robot.wobbleSpool.getTargetPosition()) <= robot.wobbletolerance) {
                telemetry.addData("pos", robot.wobbleSpool.getCurrentPosition());

                robot.wobbleSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.wobbleSpool.setTargetPosition(1000);
                robot.wobbleSpool.setPower(.5);
            } else {

                robot.wobbleSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.wobbleSpool.setTargetPosition(0);
                robot.wobbleSpool.setPower(.5);
            }
            robot.wobbleDown = true;
            if (Math.abs(robot.wobbleSpool.getCurrentPosition() - robot.wobbleSpool.getTargetPosition()) <= robot.wobbletolerance){
                telemetry.addData("Wobble", "Down");
                robot.wobbleSpool.setPower(0);}
            else {telemetry.addData("Wobble", "Up");
                robot.wobbleSpool.setPower(0);}
        } else if (!gamepad2.a) robot.wobbleDown = false;


        if(!robot.wobbleSpool.isBusy()){
            robot.wobbleSpool.setPower(0);
        }

        if (gamepad2.b && !robot.wobbleopen) {
            if (robot.wobbleGrab.getPosition() == 0.4) {
                robot.wobbleGrab.setPosition(.67);
            } else {

                robot.wobbleGrab.setPosition(.4);
            }
            robot.wobbleopen = true;
            if (robot.wobbleGrab.getPosition() == 0.4)
                telemetry.addData("Wobble2", "Opne");
            else telemetry.addData("Wobble2", "closed");
        } else if (!gamepad2.b) robot.wobbleopen = false;
*/
    }




    @Override
    public void stop() {
    }

}