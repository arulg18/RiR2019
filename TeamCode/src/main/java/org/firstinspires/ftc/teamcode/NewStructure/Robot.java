package org.firstinspires.ftc.teamcode.NewStructure;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NewStructure.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.NewStructure.Subsystems.GlyphSystem;
import org.firstinspires.ftc.teamcode.NewStructure.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.NewStructure.Subsystems.JewelSystem;
import org.firstinspires.ftc.teamcode.NewStructure.Subsystems.Relic;
import org.firstinspires.ftc.teamcode.NewStructure.Subsystems.Sensors;

public class Robot extends Systems {
    public Drivetrain drivetrain;
    public GlyphSystem glyphSystem;
    public Relic relic;
    public IMU imu;
    public JewelSystem jewelSystem;
    public Sensors sensors;

    public Systems[] allSystems;

    public Robot(HardwareMap hwMap) throws InterruptedException {
        drivetrain = new Drivetrain();
        glyphSystem = new GlyphSystem();
        relic = new Relic();
        imu = new IMU();
        jewelSystem = new JewelSystem();
        sensors = new Sensors();

        allSystems = new Systems[]{drivetrain, glyphSystem, relic, imu, jewelSystem, sensors};
        initializeHardware(hwMap);
    }

    @Override
    public boolean isFunctional() {

        for (Systems i : allSystems) {
            if (!i.isFunctional()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void initializeHardware(HardwareMap hwMap) throws InterruptedException {
        for (Systems i : allSystems) {
            i.initializeHardware(hwMap);
        }

    }
}
