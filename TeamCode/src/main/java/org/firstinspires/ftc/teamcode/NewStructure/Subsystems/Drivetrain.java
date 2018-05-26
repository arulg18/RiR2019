package org.firstinspires.ftc.teamcode.NewStructure.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NewStructure.Configuration;
import org.firstinspires.ftc.teamcode.NewStructure.Constants;
import org.firstinspires.ftc.teamcode.NewStructure.Systems;

public class Drivetrain extends Systems {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor[] driveMotors;

    public Drivetrain(){}

    @Override
    public boolean isFunctional() {
        for (DcMotor i : driveMotors)
            if (i == null) return false;
        return true;
    }

    @Override
    public void initializeHardware(HardwareMap hwMap) throws InterruptedException {

        frontLeft = motor(hwMap, Configuration.FLS, DcMotorSimple.Direction.FORWARD);
        frontRight = motor(hwMap, Configuration.FRS, DcMotorSimple.Direction.FORWARD);
        backLeft = motor(hwMap, Configuration.BLS, DcMotorSimple.Direction.FORWARD);
        backRight = motor(hwMap, Configuration.BRS, DcMotorSimple.Direction.FORWARD);

        driveMotors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};

    }
}
