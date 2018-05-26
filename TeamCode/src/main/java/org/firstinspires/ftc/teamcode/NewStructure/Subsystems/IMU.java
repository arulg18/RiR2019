package org.firstinspires.ftc.teamcode.NewStructure.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NewStructure.Systems;

public class IMU extends Systems {
    public static BNO055IMUImpl imu;
    public BNO055IMUImpl.Parameters parameters = new BNO055IMUImpl.Parameters();
    public Orientation current;
    float initorient;
    float start;
    float end;
    float xtilt;
    float ytilt;
    public static final double sensitivity = 1;
    public static boolean isnotstopped;



    @Override
    public boolean isFunctional() {
        return false;
    }

    @Override
    public void initializeHardware(HardwareMap hwMap) {

    }
}
