package org.firstinspires.ftc.teamcode.NewStructure;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Central;
import org.firstinspires.ftc.teamcode.NewStructure.Constants.*;

public abstract class NewCentral extends LinearOpMode{
    public Robot robot;
    private ElapsedTime runtime;
    private team team;

    public enum team{
        blue1, blue2, red1, red2, none;
    }
    public enum EncoderMode{
        ON, OFF
    }
    public enum turnside{
        ccw, cw
    }
    public enum axis{
        front, center, back
    }


    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }

    public void setTeam(NewCentral.team team) {
        this.team = team;
    }

    public static Location encStartLoc;
    public static float encStartAngle;

    public void motorDriveMode(EncoderMode mode, DcMotor... motor) throws InterruptedException {

        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

    }

    public static double ubl;
    public static double ubr;
    public static double ull;
    public static double ulr;

    public static final double snapSpeed = 0.05;
    public static final double xCenterOffset = 4; //VALUE TO BE CHANGED
    public static final double yCenterOffset = 4; //VALUE TO BE CHANGED

    public static final double xDistBetweenUltra = 5;
    public static final double yDistBetweenUltra = 5;
    public static final double robotLength = 18;



    static class Location{
        double x, y;
        float orient;

        public Location(double x, double y, float orient){
            this.x = x;
            this.y = y;
            this.orient = orient;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public float getOrient() {
            return orient;
        }
    }


}
