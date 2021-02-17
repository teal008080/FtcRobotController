package org.firstinspires.ftc.teamcode.robotutils;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.mastercode.UltimategoalHardware;

import static org.firstinspires.ftc.teamcode.robotutils.RobotMovement.globalAngle;

public class MathFunctions {

    public static UltimategoalHardware robot = new UltimategoalHardware();

    public static double getAngle(Orientation angles) {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        //Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle;

        return -RobotMovement.AngleWrap(deltaAngle - globalAngle);
    }

    public static void setAngle(Orientation angles){
        //Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle;
        globalAngle = deltaAngle;
    }
    /*
    public static int getColor(int sensor){

        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 500;
        if(sensor == 1) {
            Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                    (int) (robot.colorSensor.green() * SCALE_FACTOR),
                    (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
        }
        double hue = hsvValues[0];

        //hue is now the value we need to determine what color block we are looking at (light on)

        if((hue >= 35) && (hue <=55)){
            return 1; //Yellow skyblock
        }
        else if(hue > 135){
            return 2; //Black Skystone
        }
        else{
            return 0; //Nothing
        }



    }
*/
/*
    public static double getDistanceColor(int sensor){
        if(sensor == 1){
            double distance = robot.distanceSensor_color.getDistance(DistanceUnit.CM);
            //return (distance-5)*100;
            return(distance-5.475);
        }
        else{
            return 0;
        }
    }
*/

    public static double getDistance(){

        //Sensor 1 = back
        //Sensor 2 = front


            double distance = robot.dSensorFront.getDistance(DistanceUnit.INCH);
            return distance;



    }


}