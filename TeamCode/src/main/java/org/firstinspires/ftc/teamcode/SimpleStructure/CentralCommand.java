package org.firstinspires.ftc.teamcode.SimpleStructure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public abstract class CentralCommand extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private team myTeam;
    public RidgeRobot robot;



    public void setMyTeam(team myTeam) {
        this.myTeam = myTeam;
    }

    public team getMyTeam() {
        return myTeam;
    }


    public enum team{
        red1, red2, blue1, blue2, none;
    }

    public enum setupType {
        glyph, relic, jewel, drive, none, autonomous;
    }

    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }

    public void setup(team side, setupType... setupTypes){
        this.myTeam = side;
        robot = new RidgeRobot(hardwareMap, telemetry);
        ArrayList<String> setups = new ArrayList<>();
        for (setupType setup : setupTypes) {
            switch (setup){
                case glyph:
                    robot.glyph();
                    setups.add("Glyph");
                    break;
                case relic:
                    setups.add("Relic");
                    break;
                case jewel:
                    setups.add("Jewel");
                    break;
                case drive:
                    setups.add("Drive");
                    break;
                case autonomous:
                    setups.add("Autonomous");
                    break;
                case none:
                    setups.add("No Setup");
                    break;
            }
        }
        telemetry.addLine(setups.toString());
        telemetry.update();


    }
}
