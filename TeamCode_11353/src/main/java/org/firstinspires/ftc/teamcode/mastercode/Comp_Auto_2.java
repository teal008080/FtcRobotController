package org.firstinspires.ftc.teamcode.mastercode;


    import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mastercode.UltimategoalHardware;
import org.firstinspires.ftc.teamcode.robotutils.RobotMovement;
import org.firstinspires.ftc.teamcode.robotutils.MiniPID;

public class Comp_Auto_2 {
    @Autonomous(name="Phoenix Auto", group="PID")
//@Disabled
    public class OmniAutonomousPhoenixAuto extends LinearOpMode {

        public double z_angle;
        public double globalAngle;
        UltimategoalHardware robot = new UltimategoalHardware();

        MiniPID controllerAngle = new MiniPID(0.035, 0, 0.03); //.025
        MiniPID controllerDrive = new MiniPID(0.035, 0, 0); //.025
        //Past working values .035, 0, .03

        //Ziegler-Nichols standard for starting PID tuning values
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

            double deltaAngle = angles.firstAngle;

            return -RobotMovement.AngleWrap(deltaAngle - globalAngle);
        }





        public double getDistance(int sensor){
            if(sensor == 1){
                double distance = robot.dSensorFront.getDistance(DistanceUnit.CM);
                return distance;
            }

            else{
                return 0;
            }

        }

        public void drivePID(double power, double goalAngle, int direction, double goal, int sensor) {//-180 to 180
            double starTime = System.currentTimeMillis();
            controllerDrive.setOutputLimits(-1,1);
            while (true) {
                double correction = controllerDrive.getOutput(getAngle(), goalAngle);
                telemetry.addData("Distance",getDistance(2));
                telemetry.update();
                double y = -direction * power;
                double x = 0;
                double z = correction;
                robot.frontleftDrive.setPower(y + x + z);
                robot.frontrightDrive.setPower(-y + x + z);
                robot.backleftDrive.setPower(y - x + z);
                robot.backrightDrive.setPower(-y - x + z);
                if(getDistance(sensor) <= goal) {
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

        public void drivePIDtime(double power, double goalAngle, int direction, double time, int sensor) {//-180 to 180
            double starTime = System.currentTimeMillis();
            controllerDrive.setOutputLimits(-1,1);
            while (true) {
                double correction = controllerDrive.getOutput(getAngle(), goalAngle);
                telemetry.addData("Distance",getDistance(2));
                telemetry.update();
                double y = -direction * power;
                double x = 0;
                double z = correction;
                robot.frontleftDrive.setPower(y + x + z);
                robot.frontrightDrive.setPower(-y + x + z);
                robot.backleftDrive.setPower(y - x + z);
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
            robot.frontrightDrive.setPower(-power);
            robot.backrightDrive.setPower(power);
            robot.frontleftDrive.setPower(-power);
            robot.backleftDrive.setPower(power);
            sleep(time);
            stopDrive();

        }


        public void strafeRight(double power, long time){
            robot.frontrightDrive.setPower(power);
            robot.backrightDrive.setPower(-power);
            robot.frontleftDrive.setPower(power);
            robot.backleftDrive.setPower(-power);
            sleep(time);
            stopDrive();

        }


        public void turnToAnglePID(double goalAngle){//-180 to 180
            controllerAngle.setOutputLimits(-1,1);
            while (true) {

                double hotGarb = controllerAngle.getOutput(getAngle(), goalAngle);

                telemetry.addData("Angle:", getAngle()); //Gives our current pos
                telemetry.addData("Hot Garb:", hotGarb);
                telemetry.addData("Global Subtract", globalAngle);
                telemetry.update();


                hotGarb =hotGarb*robot.turnFactorPID ;

                robot.frontrightDrive.setPower(hotGarb);
                robot.backrightDrive.setPower(hotGarb);
                robot.frontleftDrive.setPower(hotGarb);
                robot.backleftDrive.setPower(hotGarb);

                if( ((goalAngle - robot.tolerancePID) <= getAngle()) && ((goalAngle + robot.tolerancePID) >= getAngle() )){
                    break;
                }

                if(getAngle() == goalAngle){
                    robot.frontrightDrive.setPower(0);
                    robot.backrightDrive.setPower(0);
                    robot.frontleftDrive.setPower(0);
                    robot.backleftDrive.setPower(0);
                    break;
                }

            }
        }

        @Override
        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap);

            waitForStart();
            setAngle();
            //Code above here should never change
            while(isStopRequested() == false) {

                drivePIDtime(.5,0,1,300,1);
                sleep(400);
                turnToAnglePID(180);
                sleep(500);
                drivePIDtime(.5,180,1,300,1);





                // Dont put code below here
                stopDrive();
                break;
            }
            stopDrive();
            stop();
        }
    }
}
