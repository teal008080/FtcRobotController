package org.firstinspires.ftc.teamcode.mastercode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robotutils.RobotMovement;
import org.firstinspires.ftc.teamcode.robotutils.MiniPID;


    @Autonomous(name="BLUE|INSIDE (AUTO)", group="PID")
//@Disabled
    public class BlueLineINSIDE_AUTO extends LinearOpMode {


        public double globalAngle;
        public double deltaAngle;
        public double error;
        public boolean busy = true;
        public double startTime;
        UltimategoalHardware robot = new UltimategoalHardware();

        MiniPID controllerAngle = new MiniPID(85, .00, 0); //.025


        //Takes the initial angle of the robot and makes that angle the zero of the program

        public void setAngle() {
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double deltaAngle = angles.firstAngle;
            globalAngle = deltaAngle;
        }

        /**
         * Get current cumulative angle rotation from last reset.
         *
         * @return Angle in degrees. + = left, - = right from zero point.
         */

        public double getAngle() {

            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            deltaAngle = angles.firstAngle;

            return -RobotMovement.AngleWrap(deltaAngle - globalAngle);
        }

        //Resets the positions of the encoders on each of the motors before each new drive method

        public void reset() {
            robot.backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        //Sets the mode of the drive motors to Run-To-Postition after the reset

        public void setRunToPosition() {
            robot.backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        //Sets the mode of the drive motors to Run-Using-Encoder after the reset

        public void setRunUsingEncoder() {
            robot.backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        //Sets a basic velocity for small movements
        public void setVel(double vel){
            robot.backrightDrive.setVelocity(vel);
            robot.backleftDrive.setVelocity(vel);
            robot.frontleftDrive.setVelocity(vel);
            robot.frontrightDrive.setVelocity(vel);

        }

        //Primary velocity setter for encoder based driving with PID angle correction

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

        //Just sets the position for the encoders to go

        public void setPos(int pos){
            robot.backrightDrive.setTargetPosition(pos);
            robot.backleftDrive.setTargetPosition(pos);
            robot.frontleftDrive.setTargetPosition(pos);
            robot.frontrightDrive.setTargetPosition(pos);


        }


        //Non-PID drive method using encoder clicks

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

        //Main drive by clicks method using the PID angle error correction

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


        //Exit loop for the drive functions

        }

        public boolean motorsBusy(){
            if(!robot.backleftDrive.isBusy()&&!robot.frontrightDrive.isBusy()&&!robot.frontleftDrive.isBusy()&&!robot.backrightDrive.isBusy()){
                busy = false;
            }   else{
                busy = true;
            }


            return busy;

        }

        //Final stop function

        public void stopDrive() {
            robot.frontleftDrive.setPower(0);
            robot.frontrightDrive.setPower(0);
            robot.backleftDrive.setPower(0);
            robot.backrightDrive.setPower(0);
        }



        //PID based turn to angle funciton

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


            //Easily usable high goal launch method

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

        //Easily usable powershot launch method


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
                turnToAnglePID(0);

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










                //End of Autonomous
                stopDrive();
                break;
            }
            stopDrive();
            stop();
        }
    }

