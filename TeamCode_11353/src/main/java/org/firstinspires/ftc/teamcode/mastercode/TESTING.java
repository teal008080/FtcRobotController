package org.firstinspires.ftc.teamcode.mastercode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mastercode.UltimategoalHardware;
import org.firstinspires.ftc.teamcode.robotutils.MathFunctions;
import org.firstinspires.ftc.teamcode.robotutils.RobotMovement;
import org.firstinspires.ftc.teamcode.robotutils.MiniPID;


@Autonomous(name="ZETA TEST1", group="PID")
//@Disabled
public class TESTING extends LinearOpMode {

    public double z_angle;
    public double globalAngle;
    public double deltaAngle;
    UltimategoalHardware robot = new UltimategoalHardware();

    MiniPID controllerAngle = new MiniPID(0.021, 0.333, 0.08);
    MiniPID controllerDrive = new MiniPID(0.00, 0, 0.00);


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

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        deltaAngle = angles.firstAngle;

        return -RobotMovement.AngleWrap(deltaAngle - globalAngle);
    }



    public void stopDrive(){
        robot.frontleftDrive.setPower(0);
        robot.frontrightDrive.setPower(0);
        robot.backleftDrive.setPower(0);
        robot.backrightDrive.setPower(0);
    }



    public void turnToAnglePID(double goalAngle){//-180 to 180
        controllerAngle.setOutputLimits(-1,1);
        controllerAngle.setOutputRampRate(.2);
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
            robot.backrightDrive.setPower(error);
            robot.frontleftDrive.setPower(-error);
            robot.backleftDrive.setPower(error);


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
        robot.shooterDrive.setPower(.75);
        sleep(5000);
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

    }



    public void reset() {
        robot.backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setModeRunTo() {
        robot.backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setPow(double power){
        robot.backrightDrive.setVelocity(power);
        robot.backleftDrive.setVelocity(power);
        robot.frontleftDrive.setVelocity(power);
        robot.frontrightDrive.setVelocity(power);

    }

    public void setPos(int pos){
        robot.backrightDrive.setTargetPosition(pos);
        robot.backleftDrive.setTargetPosition(-pos);
        robot.frontleftDrive.setTargetPosition(-pos);
        robot.frontrightDrive.setTargetPosition(pos);


    }

    public void driveByClicks(int distance, double direction, double power){
        int pos;
        pos = (int) (distance * robot.clickMult);
        reset();
        setModeRunTo();
        setPow(power);
        if (direction == 0){
            robot.backrightDrive.setTargetPosition(pos);
            robot.backleftDrive.setTargetPosition(pos);
            robot.frontleftDrive.setTargetPosition(pos);
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
            robot.backleftDrive.setTargetPosition(-pos);
            robot.frontleftDrive.setTargetPosition(-pos);
            robot.frontrightDrive.setTargetPosition(-pos);

        }
        if (direction == 3){
            robot.backrightDrive.setTargetPosition(-pos);
            robot.backleftDrive.setTargetPosition(-pos);
            robot.frontleftDrive.setTargetPosition(pos);
            robot.frontrightDrive.setTargetPosition(pos);

        }

    }


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Imu Status", robot.imu.getSystemStatus());
        telemetry.addData("Calibration Status", robot.imu.getCalibrationStatus());

        double i = 0;

        waitForStart();



        //Code above here should never change

            reset();
            robot.backrightDrive.setTargetPosition((int) (20*robot.clickMult));
            robot.backleftDrive.setTargetPosition(-(int) (20*robot.clickMult));
            robot.frontleftDrive.setTargetPosition(-(int) (20*robot.clickMult));
            robot.frontrightDrive.setTargetPosition((int) (20*robot.clickMult));
            setModeRunTo();
            setPow(45*robot.clickMult);

        while (opModeIsActive() && robot.frontrightDrive.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            i += 1;
            telemetry.addData("encoder-back-left", robot.frontrightDrive.getCurrentPosition() + "  busy=" + robot.frontrightDrive.isBusy());
            telemetry.addData("I", i);

            telemetry.update();
            idle();
        }
        setPow(0);


            // Dont put code below here


    }
}

