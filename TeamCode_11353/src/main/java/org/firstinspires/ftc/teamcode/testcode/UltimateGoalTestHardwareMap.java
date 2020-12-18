package org.firstinspires.ftc.teamcode.testcode;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.ftccommon.SoundPlayer;

public class UltimateGoalTestHardwareMap {




    /* Public OpMode members. */
    public BNO055IMU imu;
    private int monkeyID;
    private Context appContext;









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
        monkeyID = hwMap.appContext.getResources().getIdentifier("monkey", "raw", hwMap.appContext.getPackageName());
        appContext = hwMap.appContext;






    }
    void monk() {
        SoundPlayer.getInstance().startPlaying(appContext, monkeyID);

    }
}

