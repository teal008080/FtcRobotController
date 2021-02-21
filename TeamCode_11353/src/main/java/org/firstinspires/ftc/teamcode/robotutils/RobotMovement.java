
package org.firstinspires.ftc.teamcode.robotutils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mastercode.UltimategoalHardware;
import org.firstinspires.ftc.teamcode.robotutils.MathFunctions;

public class RobotMovement{

    public static double z_angle;
    public static double globalAngle;


    public static UltimategoalHardware robot = new UltimategoalHardware();



    public static MiniPID controllerAngle = new MiniPID(0.035, 0, 0.03); //.025
    public static MiniPID controllerDrive = new MiniPID(0.035, 0, 0); //.025

    public static double OpModeRuntime = System.currentTimeMillis();



    /**
     * drivePIDGeneral is a generalized straight drive with no exit condition. This means that
     * when used, the drive needs to be bound by a different condition like a color sensor
     * detection or touch sensor
     * @param power traveling velocity
     * @param goalAngle robot travel angle
     * @param direction forwards or backwards
     */
    public static void drivePIDGeneral(double power, double goalAngle, int direction) {//-180 to 180
        double starTime = System.currentTimeMillis();
        controllerDrive.setOutputLimits(-1, 1);
        while (true) {
            double correction = controllerDrive.getOutput(getAngle(), goalAngle);

            //telemetry.update();
            double y = -direction * power;
            double x = 0;
            double z = correction;
            robot.frontleftDrive.setPower(y + x + z);
            robot.frontrightDrive.setPower(-y + x + z);
            robot.backleftDrive.setPower(y - x + z);
            robot.backrightDrive.setPower(-y - x + z);
            /*
            if (isStopRequested() == true) {
                stopDrive();
                stop();
                break;
            }
            */
        }

    }

    /**
     *
     * @param power drive power
     * @param goalAngle desired robot angle
     * @param direction 1 = forward, -1 = backwards
     * @param goal distance from wall
     * @param
     */
    public static void drivePID(double power, double goalAngle, int direction, double goal, Telemetry telemetry) {//-180 to 180
        double starTime = System.currentTimeMillis();
        controllerDrive.setOutputLimits(-1, 1);
        while (true) {
            double correction = controllerDrive.getOutput(getAngle(), goalAngle);
            telemetry.addData("Distance", MathFunctions.getDistance());
            telemetry.update();
            double y = -direction * power;
            double x = 0;
            double z = correction;
            robot.frontleftDrive.setPower(y + x + z);
            robot.frontrightDrive.setPower(-y + x + z);
            robot.backleftDrive.setPower(y - x + z);
            robot.backrightDrive.setPower(-y - x + z);
            if (MathFunctions.getDistance() <= goal) {
                stopDrive();
                break;
            }
            /*
            if (isStopRequested() == true) {
                stopDrive();
                stop();
                break;
            }
            */
        }

    }

    public static void drivePIDtime(double power, double goalAngle, int direction, double time) {//-180 to 180
        double starTime = System.currentTimeMillis();
        controllerDrive.setOutputLimits(-1,1);
        while (true) {
            double correction = controllerDrive.getOutput(getAngle(), goalAngle);
            //telemetry.addData("Distance",getDistance());
            //telemetry.update();
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
            /*
            if(isStopRequested() == true){
                stopDrive();
                stop();
                break;
            }
            */
        }
    }

    public static void drivePathPID(double power, double goal) {//-180 to 180
        double starTime = System.currentTimeMillis();
        controllerDrive.setOutputLimits(-1,1);
        while (true) {
            double correctionZ = controllerAngle.getOutput(getAngle(), 0);
            double correction = controllerDrive.getOutput(MathFunctions.getDistance(), goal);
            //telemetry.update();
            double y = -1 * power;
            double x = correction;
            double z = correctionZ;
            robot.frontleftDrive.setPower(y + x + z);
            robot.frontrightDrive.setPower(-y + x + z);
            robot.backleftDrive.setPower(y - x + z);
            robot.backrightDrive.setPower(-y - x + z);
            if(MathFunctions.getDistance() <= goal) {
                stopDrive();
                break;
            }
            /*
            if(isStopRequested() == true){
                stopDrive();
                stop();
                break;
            }
            */
        }

        while (true) {
            double correctionZ = controllerAngle.getOutput(getAngle(), 0);
            double correction = controllerDrive.getOutput(MathFunctions.getDistance(), goal);
            //telemetry.update();
            double y = -1 * power;
            double x = correction;
            double z = correctionZ;
            robot.frontleftDrive.setPower(y + x + z);
            robot.frontrightDrive.setPower(-y + x + z);
            robot.backleftDrive.setPower(y - x + z);
            robot.backrightDrive.setPower(-y - x + z);
            if(MathFunctions.getDistance() <= goal) {
                stopDrive();
                break;
            }
            /*
            if(isStopRequested() == true){
                stopDrive();
                stop();
                break;
            }
            */
        }
    }

    public static void strafePID(double power, double goalAngle, int direction, double goal) {//-180 to 180
        double starTime = System.currentTimeMillis();
        controllerDrive.setOutputLimits(-1,1);
        while (true) {
            double correction = controllerDrive.getOutput(getAngle(), goalAngle);
            //telemetry.addData("Angle:", getAngle(angles)); //Gives our current pos
            //telemetry.addData("Hot Garb:", correction);
            //telemetry.addData("Global Subtract", globalAngle);
            //telemetry.update();
            double y = -direction*power;
            double x = 0;
            double z = correction;
            robot.frontleftDrive.setPower(-y + x + z);
            robot.frontrightDrive.setPower(-y + x + z);
            robot.backleftDrive.setPower(y - x + z);
            robot.backrightDrive.setPower(y - x + z);
            if (MathFunctions.getDistance() <= goal){
                stopDrive();
                break;
            }
            /*
            if(isStopRequested() == true){
                stopDrive();
                stop();
                break;
            }
            */

        }
    }

    public static void stopDrive(){
        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);
    }

    public static void strafeLeft(double power, double time){
        double starTime = System.currentTimeMillis();
        robot.frontrightDrive.setPower(-power);
        robot.backrightDrive.setPower(power);
        robot.frontleftDrive.setPower(-power);
        robot.backleftDrive.setPower(power);
        if(System.currentTimeMillis()-starTime >= time) {
            stopDrive();
        }

    }

    public static void strafeRight(double power, double time){
        double starTime = System.currentTimeMillis();
        robot.frontrightDrive.setPower(power);
        robot.backrightDrive.setPower(-power);
        robot.frontleftDrive.setPower(power);
        robot.backleftDrive.setPower(-power);
        if(System.currentTimeMillis()-starTime >= time) {
            stopDrive();
        }
    }

    public static void turnToAnglePID(double goalAngle, Telemetry telemetry){//-180 to 180
        controllerAngle.setOutputLimits(-1,1);
        while (true) {
            telemetry.addData("Turntoanglepre", "Yes");
            telemetry.update();
            double hotGarb = controllerAngle.getOutput(getAngle(), goalAngle);
            telemetry.addData("tuentoanglepost", "yes");

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

    /**
     * AngleWrap makes sure the angle the robot is at is between -180 and 180 ( - PI to PI)
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while(angle < -180){
            angle += 360;
        }
        while(angle > 180){
            angle -= 360;
        }
        return angle;
    }
    public static double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle;

        return -RobotMovement.AngleWrap(deltaAngle - globalAngle);
    }


    /**
     * This Array List draws a circle and where that circle intersects a line (drawn by the target of the robot designated by teamcode.RobotMovement.goToPosition) it marks a point.
     * @param circleCenter
     * @param radius
     * @param linePoint1
     * @param linePoint2
     * @return
     */
    /*
    public static ArrayList<Point> lineCircleIntersection (Point circleCenter, double radius, Point linePoint1, Point linePoint2){
        //This avoids vertical and horizontal lines by eliminating any slopes that near 0 or infinity
        if(Math.abs(linePoint1.y - linePoint2.y) < .003){
            linePoint1.y = linePoint2.y + .003;
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < .003){
            linePoint1.x = linePoint2.x + .003;
        }
        //Rise over Run to get slope of line
        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x - linePoint1.x);
        //Math to set up quadratic formula
        double quadraticA = 1.0 + Math.pow(m1,2);
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);
        double quadraticC = ((Math.pow(m1,2) * Math.pow(x1,2))) - (2.0*y1*m1*x1) + Math.pow(y1,2)-Math.pow(radius,2);
        ArrayList<Point> allPoints = new ArrayList<>();
        try{
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            //Put back the offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1,yRoot1));
            }
            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;
            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot2,yRoot2));
            }
        }catch(Exception e){
        }
        return allPoints;
    }*/
}