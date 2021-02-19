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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Disabled
public class UltimategoalHardware {

    /* Public OpMode members. */
    public BNO055IMU       imu;

    public long waittime = 0;

    public DcMotor  frontleftDrive      = null;
    public DcMotor  frontrightDrive     = null;
    public DcMotor  backrightDrive      = null;
    public DcMotor  backleftDrive       = null;

    public DcMotor  intakeChainDrive    = null;
    public DcMotor  shooterDrive        = null;

    public Servo    drop                = null;
    public Servo    triggerServo             = null;

    public DistanceSensor dSensorBack = null;
    public DistanceSensor dSensorFront = null;

    public DcMotor wobbleSpool = null;
    public Servo wobbleGrab = null;



    public int      speedFactor         = 1;
    public int      reverseFactor       = 1;

    public boolean intakeToggle = false;
    public boolean intake = false;

    public boolean triggerToggle = true;
    public boolean trigger = false;

    public boolean shooterToggle = true;
    public boolean shooter = false;

    public boolean wobbleDown = true;





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


        // Define and Initialize Motors

        frontleftDrive        = hwMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive       = hwMap.get(DcMotor.class, "front_right_drive");
        backleftDrive         = hwMap.get(DcMotor.class, "back_left_drive");
        backrightDrive        = hwMap.get(DcMotor.class, "back_right_drive");
        intakeChainDrive      = hwMap.get(DcMotor.class, "chain_drive");
        shooterDrive          = hwMap.get(DcMotor.class, "shooter_drive");
        //wobbleSpool           = hwMap.get(DcMotor.class, "wobblespool");

        drop                  =hwMap.get(Servo.class, "drop");
        triggerServo               =hwMap.get(Servo.class, "trigger");
        //wobbleGrab             = hwMap.get(Servo.class, "grab");

        intakeChainDrive.setDirection(DcMotor.Direction.REVERSE);
        shooterDrive.setDirection(DcMotor.Direction.REVERSE);
        frontleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backleftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        intakeChainDrive.setPower(0);
        shooterDrive.setPower(0);
        wobbleSpool.setPower(.3);

        //wobbleGrab.setPosition(0);
        drop.setPosition(.48);
        triggerServo.setPosition(.44);

       // dSensorBack        = hwMap.get(DistanceSensor.class, "distance_sensor");
        //dSensorFront      = hwMap.get(DistanceSensor.class, "distance_sensor_front");



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Chain intake drive





    }
}
