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


    @Autonomous(name="Ultimate Goal Auto Blue, Right Line", group="PID")
//@Disabled
    public class Comp_Auto_2 extends LinearOpMode {

        public double z_angle;
        public double globalAngle;
        public double deltaAngle;
        UltimategoalHardware robot = new UltimategoalHardware();

        MiniPID controllerAngle = new MiniPID(.02, .05,.02); //.025
        MiniPID controllerDrive = new MiniPID(0.01, 0.0, 0.01); //.025
        //Past working values .035, 0, .03

        //Ziegler-Nichols standard for starting PID tuning value
        //Kcr = Proportional gain that causes steady osscillation (.04)
        //Pcr = Period of Kcr's Oscillation (measured in seconds) (1.4s) T
        //In a full PID system:
        //Proportional: .8Kcr
        //Derivative: (Ku*Tu)/10

        // called when init button is  pressed.

        /**
         * Get current cumulative angle rotation from last reset.
         * @return Angle in degrees. + = left, - = right from zero point.
         */

        public void setAngle(){
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double deltaAngle = angles.firstAngle;
            globalAngle = deltaAngle;
        }

        public double getAngle() {
            // We experimentally determined the Z axis is the axis we want to use for heading angle.
            // We have to process the angle because the imu works in euler angles so the Z axis is
            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            deltaAngle = angles.firstAngle;

            return -RobotMovement.AngleWrap(deltaAngle - globalAngle);
        }

        public boolean isirregular(){
            boolean irregular = false;
            double starttime = System.currentTimeMillis();
            double expected = starttime/22;
            double current = robot.intakeChainDrive.getCurrentPosition();
            if (expected - current != 0) {
                irregular = true;
            }


          return irregular;
        }





        public double getDistance(){

                double distance = robot.dSensorFront.getDistance(DistanceUnit.CM);
                return distance;




        }

        public void drivePID(double power, double goalAngle, int direction, double goal) {//-180 to 180
            double starTime = System.currentTimeMillis();
            controllerDrive.setOutputLimits(-1,1);
            while (true) {
                double correction = controllerDrive.getOutput(getAngle(), goalAngle);
                telemetry.addData("Distance",getDistance());
                telemetry.addData("Error:", correction);
                telemetry.update();
                double y = -direction * power;
                double x = 0;
                double z = correction;
                z = z * robot.turnFactorPID;
                robot.frontleftDrive.setPower(y + x - z);
                robot.frontrightDrive.setPower(-y + x + z);
                robot.backleftDrive.setPower(y - x - z);
                robot.backrightDrive.setPower(-y - x + z);

               double abserror = Math.abs(getDistance()-goal);
                if(abserror <= robot.tolerancePID2) {
                    stopDrive();
                    break;
                }
                if(isStopRequested() == true){
                    stopDrive();
                    stop();
                    break;
                }
            }
        }

        public void drivePIDtime(double power, double goalAngle, int direction, double time) {//-180 to 180
            double starTime = System.currentTimeMillis();
            controllerDrive.setOutputLimits(-1,1);
            while (true) {
                double correction = controllerDrive.getOutput(getAngle(), goalAngle);
                telemetry.addData("Distance",getDistance());
                telemetry.addData("Error:", correction);
                telemetry.update();
                double y = -direction * power;
                double x = 0;
                double z = correction;
                z = z * robot.turnFactorPID;
                robot.frontleftDrive.setPower(y + x - z);
                robot.frontrightDrive.setPower(-y + x + z);
                robot.backleftDrive.setPower(y - x - z);
                robot.backrightDrive.setPower(-y - x + z);
                if(System.currentTimeMillis()-starTime >= time) {
                    stopDrive();
                    break;
                }
                if(isStopRequested() == true){
                    stopDrive();
                    stop();
                    break;
                }
            }
        }

        public void strafePID(double power, double goalAngle, int direction, double goal) {//-180 to 180
            double starTime = System.currentTimeMillis();
            controllerDrive.setOutputLimits(-1,1);
            while (true) {
                double correction = controllerDrive.getOutput(getAngle(), goalAngle);
                telemetry.addData("Angle:", getAngle()); //Gives our current pos
                telemetry.addData("Hot Garb:", correction);
                telemetry.addData("Global Subtract", globalAngle);
                telemetry.update();
                double y = -direction*power;
                double x = 0;
                double z = correction;
                robot.frontleftDrive.setPower(-y + x + z);
                robot.frontrightDrive.setPower(-y + x + z);
                robot.backleftDrive.setPower(y - x + z);
                robot.backrightDrive.setPower(y - x + z);

                if(isStopRequested() == true){
                    stopDrive();
                    stop();
                    break;
                }

            }
        }

        public void stopDrive(){
            robot.frontleftDrive.setPower(0);
            robot.frontrightDrive.setPower(0);
            robot.backleftDrive.setPower(0);
            robot.backrightDrive.setPower(0);
        }





        public void strafeLeft(double power, long time){
            robot.frontrightDrive.setPower(power);
            robot.backrightDrive.setPower(-power);
            robot.frontleftDrive.setPower(power);
            robot.backleftDrive.setPower(-power);
            sleep(time);
            stopDrive();

        }


        public void strafeRight(double power, long time){
            robot.frontrightDrive.setPower(-power);
            robot.backrightDrive.setPower(power);
            robot.frontleftDrive.setPower(-power);
            robot.backleftDrive.setPower(power);
            sleep(time);
            stopDrive();

        }


        public void turnToAnglePID(double goalAngle){//-180 to 180
            controllerAngle.setOutputLimits(-1,1);

            while (true) {
                getAngle();
                double error = controllerAngle.getOutput(getAngle(), goalAngle);


                telemetry.addData("Angle:", getAngle()); //Gives our current pos
                telemetry.addData("Error:", error);
                telemetry.addData("Global Subtract", globalAngle);
                telemetry.addData("Goal", goalAngle);
                telemetry.update();


                error = error*robot.turnFactorPID ;

                robot.frontrightDrive.setPower(-error);
                robot.backrightDrive.setPower(-error);
                robot.frontleftDrive.setPower(-error);
                robot.backleftDrive.setPower(-error);


                double abserr = Math.abs(getAngle() - goalAngle);

                if(abserr <= robot.tolerancePID){
                    robot.frontrightDrive.setPower(0);
                    robot.backrightDrive.setPower(0);
                    robot.frontleftDrive.setPower(0);
                    robot.backleftDrive.setPower(0);
                    break;
                }

            }


        }
        public void launch3shots() {
            robot.shooterDrive.setPower(.78);
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
            robot.shooterDrive.setPower(0);
        }
        public void launch3powershots() {
            robot.shooterDrive.setPower(.67);
            sleep(2000);
            robot.triggerServo.setPosition(.55);
            sleep(250);
            robot.triggerServo.setPosition(.43);
            sleep(250);
            strafeRight(.5,200);
            robot.triggerServo.setPosition(.55);
            sleep(250);
            robot.triggerServo.setPosition(.43);
            sleep(250);
            strafeRight(.5,200);
            robot.triggerServo.setPosition(.55);
            sleep(250);
            robot.triggerServo.setPosition(.43);
            robot.shooterDrive.setPower(0);
            strafeLeft(.5,200);
            sleep(250);
        }


        //Assumes the robot is at the side to the left of the blue tower, looking at the tower. Facing the back wall.
        public void ringIntakeSweep() {
            robot.intakeChainDrive.setPower(1);
            while (!isirregular()) {
                drivePID(.3, 0, 1, 30);
                turnToAnglePID(-0);
                drivePID(.3, -0, 1, 30);
                turnToAnglePID(180);
                drivePID(.3, 180, 1, 30);
                turnToAnglePID(0);
                drivePID(.3, 0, 1, 30);
                turnToAnglePID(0);
            }
            robot.intakeChainDrive.setPower(0);

        }

        public void wobbledrop() {
            robot.wobbleSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobbleSpool.setPower(1);
            robot.wobbleSpool.setTargetPosition(1000);
            robot.wobbleGrab.setPosition(.67);
            sleep(500);


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

            //Code above here should never change
            while(!isStopRequested()) {
                drivePIDtime(.8,0,-1,250);
                sleep(350);
                strafeLeft(.3,600);//
                sleep(350);
                drivePIDtime(1,0,-1,800);
                sleep(350);
                //turnToAnglePID(0);
                sleep(200);
                launch3powershots();

                sleep(350);
                drivePIDtime(1,0,-1,1150);
                sleep(350);
                strafeRight(.8,700 );

                turnToAnglePID(178);
                sleep(350);
                wobbledrop();
                turnToAnglePID(-0);
                robot.drop.setPosition(.6);

                strafeLeft(.6,220);
                turnToAnglePID(0);
                sleep(400);
                robot.intakeChainDrive.setPower(1);
                sleep(350);
                drivePIDtime(1,-0,1,1800);
                sleep(350);

                sleep(350);

                drivePIDtime(1,-0,-1,550);
                robot.intakeChainDrive.setPower(0);
                sleep(350);
                launch3shots();
                sleep(350);
                drivePIDtime(1,-0,-1,200);
                sleep(300);
                strafeLeft(1,300);









                // Dont put code below here
                stopDrive();
                break;
            }
            stopDrive();
            stop();
        }
    }

