package org.firstinspires.ftc.teamcode.mastercode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mastercode.UltimategoalHardware;
import org.firstinspires.ftc.teamcode.robotutils.MathFunctions;
import org.firstinspires.ftc.teamcode.robotutils.RobotMovement;
import org.firstinspires.ftc.teamcode.robotutils.MiniPID;


    @Autonomous(name="BLUE|INSIDE (AUTO)", group="PID")
//@Disabled
    public class BlueLineINSIDE_AUTO extends LinearOpMode {

        public double z_angle;
        public double globalAngle;
        public double deltaAngle;
        public double error;
        public boolean busy = true;
        public double startTime;
        UltimategoalHardware robot = new UltimategoalHardware();

        MiniPID controllerAngle = new MiniPID(85, .00, 0); //.025
        MiniPID controllerDrive = new MiniPID(0.01, 0.0, 0.01); //.025


        public void setAngle() {
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double deltaAngle = angles.firstAngle;
            globalAngle = deltaAngle;
        }

        public double getAngle() {

            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            deltaAngle = angles.firstAngle;

            return -RobotMovement.AngleWrap(deltaAngle - globalAngle);
        }

        public void reset() {
            robot.backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

        public void setRunToPosition() {
            robot.backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        public void setRunUsingEncoder() {
            robot.backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        public void setVel(double vel){
            robot.backrightDrive.setVelocity(vel);
            robot.backleftDrive.setVelocity(vel);
            robot.frontleftDrive.setVelocity(vel);
            robot.frontrightDrive.setVelocity(vel);

        }
        public void setVelWithError(double vel, double goalAngle, double direction){

            controllerAngle.setOutputLimits(-50*robot.clickMult,50*robot.clickMult);
            error = controllerAngle.getOutput(getAngle(),goalAngle);

            if (direction == 0){
                robot.backrightDrive.setVelocity(vel-error);
                robot.backleftDrive.setVelocity(vel+error);
                robot.frontleftDrive.setVelocity(vel+error);
                robot.frontrightDrive.setVelocity(vel-error);
            }
            if (direction == 1){
                robot.backrightDrive.setVelocity(vel-error);
                robot.backleftDrive.setVelocity(vel-error);
                robot.frontleftDrive.setVelocity(vel+error);
                robot.frontrightDrive.setVelocity(vel+error);

            }
            if (direction == 2){
                robot.backrightDrive.setVelocity(vel+error);
                robot.backleftDrive.setVelocity(vel-error);
                robot.frontleftDrive.setVelocity(vel-error);
                robot.frontrightDrive.setVelocity(vel+error);

            }
            if (direction == 3){
                robot.backrightDrive.setVelocity(vel+error);
                robot.backleftDrive.setVelocity(vel+error);
                robot.frontleftDrive.setVelocity(vel-error);
                robot.frontrightDrive.setVelocity(vel-error);

            }



        }

        public void setPos(int pos){
            robot.backrightDrive.setTargetPosition(pos);
            robot.backleftDrive.setTargetPosition(pos);
            robot.frontleftDrive.setTargetPosition(pos);
            robot.frontrightDrive.setTargetPosition(pos);


        }




        public void driveByClicks(int distance, double direction, double vel){
            int pos;
            pos = (int) (distance * robot.clickMult);
            reset();
            if (direction == 0){
                robot.backrightDrive.setTargetPosition(pos);
                robot.backleftDrive.setTargetPosition(-pos);
                robot.frontleftDrive.setTargetPosition(-pos);
                robot.frontrightDrive.setTargetPosition(pos);
            }
            if (direction == 1){
                robot.backrightDrive.setTargetPosition(pos);
                robot.backleftDrive.setTargetPosition(pos);
                robot.frontleftDrive.setTargetPosition(-pos);
                robot.frontrightDrive.setTargetPosition(-pos);

            }
            if (direction == 2){
                robot.backrightDrive.setTargetPosition(-pos);
                robot.backleftDrive.setTargetPosition(pos);
                robot.frontleftDrive.setTargetPosition(pos);
                robot.frontrightDrive.setTargetPosition(-pos);

            }
            if (direction == 3){
                robot.backrightDrive.setTargetPosition(-pos);
                robot.backleftDrive.setTargetPosition(-pos);
                robot.frontleftDrive.setTargetPosition(pos);
                robot.frontrightDrive.setTargetPosition(pos);

            }

            vel = vel*robot.clickMult;
            setRunToPosition();
            setVel(vel);


            while (opModeIsActive() && robot.frontrightDrive.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            {

                telemetry.addData("Front Right Position", robot.frontrightDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.frontrightDrive.isBusy());
                telemetry.addData("Front Left Position", robot.frontleftDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.frontleftDrive.isBusy());
                telemetry.addData("Back Right Position", robot.backrightDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.backrightDrive.isBusy());
                telemetry.addData("Back Left Position", robot.backleftDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.backleftDrive.isBusy());


                telemetry.update();

            }
            setVel(0);



        }
        public void driveByClicksPID(double distance, double direction, double vel, double goalAngle){
            int pos;
            pos = (int) (distance * robot.clickMult);
            reset();
            if (direction == 0){
                robot.backrightDrive.setTargetPosition(pos);
                robot.backleftDrive.setTargetPosition(-pos);
                robot.frontleftDrive.setTargetPosition(-pos);
                robot.frontrightDrive.setTargetPosition(pos);
            }
            if (direction == 1){
                robot.backrightDrive.setTargetPosition(pos);
                robot.backleftDrive.setTargetPosition(pos);
                robot.frontleftDrive.setTargetPosition(-pos);
                robot.frontrightDrive.setTargetPosition(-pos);

            }
            if (direction == 2){
                robot.backrightDrive.setTargetPosition(-pos);
                robot.backleftDrive.setTargetPosition(pos);
                robot.frontleftDrive.setTargetPosition(pos);
                robot.frontrightDrive.setTargetPosition(-pos);

            }
            if (direction == 3){
                robot.backrightDrive.setTargetPosition(-pos);
                robot.backleftDrive.setTargetPosition(-pos);
                robot.frontleftDrive.setTargetPosition(pos);
                robot.frontrightDrive.setTargetPosition(pos);

            }

            vel = vel*robot.clickMult;
            setRunToPosition();
            setVel(vel);


            while (opModeIsActive() && motorsBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            {


                setVelWithError(vel, goalAngle, direction);
                telemetry.addData("Front Right Position", robot.frontrightDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.frontrightDrive.isBusy());
                telemetry.addData("Front Left Position", robot.frontleftDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.frontleftDrive.isBusy());
                telemetry.addData("Back Right Position", robot.backrightDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.backrightDrive.isBusy());
                telemetry.addData("Back Left Position", robot.backleftDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.backleftDrive.isBusy());
                telemetry.addData("Error", error);
                telemetry.addData("angel", getAngle());

                telemetry.update();



                if(!robot.frontrightDrive.isBusy()){
                    robot.frontrightDrive.setVelocity(0);
                }
                if(!robot.backrightDrive.isBusy()){
                    robot.backrightDrive.setVelocity(0);
                }
                if(!robot.frontleftDrive.isBusy()){
                    robot.frontleftDrive.setVelocity(0);
                }
                if(!robot.backleftDrive.isBusy()){
                    robot.backleftDrive.setVelocity(0);
                }



            }
            setVel(0);




        }

        public boolean motorsBusy(){
            if(!robot.backleftDrive.isBusy()&&!robot.frontrightDrive.isBusy()&&!robot.frontleftDrive.isBusy()&&!robot.backrightDrive.isBusy()){
                busy = false;
            }   else{
                busy = true;
            }


            return busy;

        }

        public void stopDrive() {
            robot.frontleftDrive.setPower(0);
            robot.frontrightDrive.setPower(0);
            robot.backleftDrive.setPower(0);
            robot.backrightDrive.setPower(0);
        }





        public void turnToAnglePID(double goalAngle){//-180 to 180
            reset();
            setRunUsingEncoder();
            double startime = System.currentTimeMillis();
            controllerAngle.setOutputLimits(-50*robot.clickMult,50*robot.clickMult);

            while (true) {
                getAngle();
                double error = controllerAngle.getOutput(getAngle(), goalAngle);


                telemetry.addData("Angle:", getAngle()); //Gives our current pos
                telemetry.addData("Error:", error);
                telemetry.addData("Global Subtract", globalAngle);
                telemetry.addData("Goal", goalAngle);
                telemetry.update();




                robot.frontrightDrive.setVelocity(-error);
                robot.backrightDrive.setVelocity(-error);
                robot.frontleftDrive.setVelocity(-error);
                robot.backleftDrive.setVelocity(-error);




                if(Math.abs(getAngle()-goalAngle) <= 0.1){
                    setVel(0);
                    break;
                }

            }


        }
        public void launch3shots() {
            robot.shooterDrive.setVelocity(29*robot.clickMult);
            sleep(2000);
            robot.triggerServo.setPosition(.55);
            sleep(200);
            robot.triggerServo.setPosition(.43);
            sleep(200);
            robot.triggerServo.setPosition(.55);
            sleep(200);
            robot.triggerServo.setPosition(.43);
            sleep(200);
            robot.triggerServo.setPosition(.55);
            sleep(200);
            robot.triggerServo.setPosition(.43);
            robot.shooterDrive.setVelocity(0);
        }
        public void launch3powershots() {
            robot.shooterDrive.setVelocity(26*robot.clickMult);
            robot.triggerServo.setPosition(.55);
            sleep(250);
            robot.triggerServo.setPosition(.43);
            sleep(250);
            driveByClicksPID(7.5,1,6,0);
            robot.triggerServo.setPosition(.55);
            sleep(250);
            robot.triggerServo.setPosition(.43);
            sleep(250);
            driveByClicksPID(7.5,1,6,0);
            robot.triggerServo.setPosition(.55);
            sleep(250);
            robot.triggerServo.setPosition(.43);
            robot.shooterDrive.setVelocity(0);

            sleep(250);
        }







        @Override
        public void runOpMode(){
            robot.init(hardwareMap);
            telemetry.addData("Imu Status", robot.imu.getSystemStatus());
            telemetry.addData("Calibration Status", robot.imu.getCalibrationStatus());
            robot.wobbleSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobbleGrab.setPosition(.4);
            waitForStart();
            setAngle();
            startTime = System.currentTimeMillis();
            //Code above here should never change
            while(!isStopRequested()) {

                driveByClicksPID(24,2,40,0);
                sleep(300);
                driveByClicksPID(20,3,40,0);
                sleep(300);
                driveByClicksPID(35,2,40,0);
                robot.shooterDrive.setVelocity(26*robot.clickMult);
                turnToAnglePID(0);

                sleep(800);
                launch3powershots();
                sleep(300);
                driveByClicksPID(16,1,40,0);
                sleep(300);
                robot.intakeChainDrive.setPower(1);
                sleep(300);
                driveByClicksPID(19,0,35,0);
                sleep(300);
                driveByClicksPID(19,2,40,0);
                sleep(300);
                robot.intakeChainDrive.setPower(0);
                robot.shooterDrive.setVelocity(29*robot.clickMult);

                turnToAnglePID(0);
                launch3shots();
                driveByClicksPID(15,2,40,0);
                sleep(300);
                driveByClicksPID(20,3,40,0);
                turnToAnglePID(0);
                sleep(300);










                // Dont put code below here
                stopDrive();
                break;
            }
            stopDrive();
            stop();
        }
    }

