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


@Autonomous(name="ZETA TEST2", group="PID")
//@Disabled
public class TESTING2 extends LinearOpMode {

    public double z_angle;
    public double globalAngle;
    public double deltaAngle;
    UltimategoalHardware robot = new UltimategoalHardware();

    MiniPID controllerAngle = new MiniPID(100, .00, 0); //.025
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
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */

    public double error;

    public void setAngle() {
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

    public void reset() {
        robot.backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setCondom() {
        robot.backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setVel(double vel){
        robot.backrightDrive.setVelocity(vel);
        robot.backleftDrive.setVelocity(vel);
        robot.frontleftDrive.setVelocity(vel);
        robot.frontrightDrive.setVelocity(vel);

    }

    public void setPos(int pos){
        robot.backrightDrive.setTargetPosition(pos);
        robot.backleftDrive.setTargetPosition(pos);
        robot.frontleftDrive.setTargetPosition(pos);
        robot.frontrightDrive.setTargetPosition(pos);


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




    public double getDistance() {

        double distance = robot.dSensorFront.getDistance(DistanceUnit.CM);
        return distance;


    }

    public void driveByClicksPID(int distance, double direction, double vel, double goalAngle){
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
        setCondom();
        setVel(vel);


        while (opModeIsActive() && robot.frontrightDrive.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            setVelWithError(vel, goalAngle, direction);
            telemetry.addData("Front Right Position", robot.frontrightDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.frontrightDrive.isBusy());
            telemetry.addData("Front Left Position", robot.frontleftDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.frontleftDrive.isBusy());
            telemetry.addData("Back Right Position", robot.backrightDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.backrightDrive.isBusy());
            telemetry.addData("Back Left Position", robot.backleftDrive.getCurrentPosition()/robot.clickMult + "  busy=" + robot.backleftDrive.isBusy());
            telemetry.addData("Error", error);
            telemetry.addData("angel", getAngle());
            telemetry.update();

        }
        setVel(0);



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
        setCondom();
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



    public void stopDrive() {
        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);
    }




    public void strafeRight(double power, long time, double goalAngle) {
        controllerDrive.setOutputLimits(-1,1);
        while (true) {
            double correction = controllerAngle.getOutput(getAngle(), goalAngle);

            telemetry.addData("Hot Garb:", correction);
            double z = correction;

            robot.frontrightDrive.setPower(-power - z);
            robot.backrightDrive.setPower(power + z);
            robot.frontleftDrive.setPower(-power + z);
            robot.backleftDrive.setPower(power - z);


            sleep(time);
            stopDrive();
            break;

        }

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
        robot.shooterDrive.setPower(.65);
        sleep(2500);
        robot.triggerServo.setPosition(.55);
        sleep(250);
        robot.triggerServo.setPosition(.43);
        sleep(250);
        driveByClicks(7,1,6);
        robot.triggerServo.setPosition(.55);
        sleep(250);
        robot.triggerServo.setPosition(.43);
        sleep(250);
        driveByClicks(7,1,6);
        robot.triggerServo.setPosition(.55);
        sleep(250);
        robot.triggerServo.setPosition(.43);
        robot.shooterDrive.setPower(0);

        sleep(250);
    }


    //Assumes the robot is at the side to the left of the blue tower, looking at the tower. Facing the back wall.


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

              driveByClicksPID(500,1,30,0);
              sleep(500);
              driveByClicksPID(500,3,30,0);









            // Dont put code below here
            stopDrive();
            break;
        }
        stopDrive();
        stop();
    }
}

