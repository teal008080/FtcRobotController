package org.firstinspires.ftc.test_teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class UltimateGoalTestHardwareMap {




    /* Public OpMode members. */
    public BNO055IMU imu;




    public DcMotor conveyerDrive = null;
    //public DcMotor shooterDrive = null;








    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public UltimateGoalTestHardwareMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;





        //SHOOTER HARDWARE

        conveyerDrive        = hwMap.get(DcMotor.class, "conveyerDrive");
        //shooterDrive       = hwMap.get(DcMotor.class, "shooterDrive");

        conveyerDrive.setPower(0);
       // shooterDrive.setPower(0);

    }
}

