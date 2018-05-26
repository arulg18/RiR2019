package org.firstinspires.ftc.teamcode.NewStructure.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewStructure.Configuration;
import org.firstinspires.ftc.teamcode.NewStructure.Systems;

public class JewelSystem extends Systems {
    public Servo downServo;
    public Servo sweepServo;

    public Servo[] servos;

    @Override
    public boolean isFunctional() {
        for (Servo i : servos)
            if (i == null)
                return false;
        return false;
    }

    @Override
    public void initializeHardware(HardwareMap hwMap) throws InterruptedException {
        downServo = servo(hwMap, Configuration.downServoS, Servo.Direction.FORWARD, 0, 1, 0);
        sweepServo = servo(hwMap, Configuration.sweepServoS, Servo.Direction.FORWARD, 0, 1, 0);

        servos = new Servo[]{downServo, sweepServo};

    }
}
