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



@Disabled
public class UltimategoalHardware {

    /* Public OpMode members. */
    public BNO055IMU       imu;

    public DcMotor frontleftdrive = null;
    public DcMotor frontrightdrive = null;
    public DcMotor backleftdrive = null;
    public DcMotor Gary = null;
    public DcMotor shooter_motor = null;




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

        //Define and Initialize Sensors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        // Define and Initialize Motors

        frontleftdrive = hwMap.get(DcMotor.class, "frontleftdrive");
        frontrightdrive = hwMap.get(DcMotor.class, "frontrightdrive");
        backleftdrive = hwMap.get(DcMotor.class, "backleftdrive");
        Gary = hwMap.get(DcMotor.class, "backrightdrive");
        shooter_motor = hwMap.get(DcMotor.class, "shooter_motor");
        frontleftdrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightdrive.setDirection(DcMotor.Direction.REVERSE);
        backleftdrive.setDirection(DcMotor.Direction.REVERSE);
        Gary.setDirection(DcMotor.Direction.REVERSE);
        shooter_motor.setPower(0);

        frontleftdrive.setPower(0);
        frontrightdrive.setPower(0);
        backleftdrive.setPower(0);
        Gary.setPower(0);
        shooter_motor.setPower(0);

        frontleftdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Gary.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}
