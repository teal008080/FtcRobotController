/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode.mastercode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
@Disabled
public class UltimategoalHardware {

    /* Public OpMode members. */
    public BNO055IMU       imu;



    public DcMotor  frontleftDrive      = null;
    public DcMotor  frontrightDrive     = null;
    public DcMotor  backrightDrive      = null;
    public DcMotor  backleftDrive       = null;



    public int      speedFactor         = 1;



    /*public double      DDRD                = .65;
    public double      DDLD                = .55;
    public double      DDRI                =   1;
    public double      DDLI                = .9;

    public double     delivIdle            = .6;
    public double     delivGrab            = .91;

    public double     turnFactorPID        = .5;

    public double     tolerancePID         = 2;
    public double     tolerancePID_d       = 1;
    */


    //public static MiniPID controllerAngle = new MiniPID(0.035, 0, 0.03); //.025
    //public static MiniPID controllerDrive = new MiniPID(0.035, 0, 0); //.025

    /*
       We use cad on the team in many different ways. One of those ways is through
       creating custom parts like our auto claw, to maximize the ability of our robot.
       We also use cad to design our schools theatre set and make awards for band.
       With these things we are able to help the extracurriculars. +


       Not only have we been able to implement cad into our rorbot design for custom parts.
       but weve also been able to implement cad into our fundrasing, outreaching into our
       theatre program and outreaching into our band program to host a fun awards night.
    */

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public UltimategoalHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;



       /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hwMap.get(BNO055IMU.class, "imu");
        //imu2 = hwMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);


        //imu2.initialize(parameters);
/*
        while(!imu.isGyroCalibrated() && !imu2.isGyroCalibrated()){
            //I don't know how necessary this is, decided to include it
        }
*/
        // Define and Initialize Motors

        frontleftDrive        = hwMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive       = hwMap.get(DcMotor.class, "front_right_drive");
        backleftDrive         = hwMap.get(DcMotor.class, "back_left_drive");
        backrightDrive        = hwMap.get(DcMotor.class, "back_right_drive");


        frontleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }
}
