package org.firstinspires.ftc.teamcode.Functionality;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Central;

/**
 * Created by Administrator on 04/24/2018.
 */

public class RangePrediction extends Central {
    public static final int interval = 100;
    public static final double XSensorOffset =2; //between sensors and the center
    public static final double XSensorDist=2;//between sensors
    public static final double YSensorOffset =2;
    public static final double YSensorDist=2;
    public static final int RangeTolerance = 2;

    //btw this program assumes the center of the robot is at the intersections of the perpendicular bisectors of the Sensors but i can change that if needed

    //Changing Variables:
    public static int[] Pos;
    public static int[] PosOld;
    public static boolean IsUsingRange =true;

    public enum rangefinderid{
        xback,xfront,yleft,yright
    }
    public void StartAbsPosition(){
        new Thread(new RangeDaemon()).start();
    }
    static public void SmartPos (){ //output is given to Pos and PosOld public vars
        if(Math.abs(getPosfromRange()[0]-PosOld[0])+Math.abs(getPosfromRange()[1]-PosOld[1])<RangeTolerance)
        {
            Pos = getPosfromRange();
            IsUsingRange = true;
        }
        else if(IsUsingRange)
        {
            Velocity velo = new Velocity(DistanceUnit.INCH, (double)PosOld[0],(double)PosOld[1],0,0);
            imu.startAccelerationIntegration(new Position(DistanceUnit.INCH,(double)PosOld[0],(double)PosOld[1],0,0),velo,10);
            IsUsingRange =false;
        }
        else
        {
            Pos[0] = (int) imu.getPosition().x;
            Pos[1] = (int) imu.getPosition().y;
        }

        PosOld[0] = Pos[0];
        PosOld[1] = Pos[1];
    }
    static public int[] getPosfromRange() {
        double angleX = Math.atan((xfront.getDistance(DistanceUnit.INCH)-xback.getDistance(DistanceUnit.INCH))/XSensorDist);
        double angleY = Math.atan((yleft.getDistance(DistanceUnit.INCH)-yright.getDistance(DistanceUnit.INCH))/YSensorDist);
        double angle = (angleX+angleY)/2;
        double x = Math.cos(angle)*(XSensorOffset+((xfront.getDistance(DistanceUnit.INCH)+xfront.getDistance(DistanceUnit.INCH))/2));
        double y = Math.cos(angle)*(YSensorOffset+((yright.getDistance(DistanceUnit.INCH)+yleft.getDistance(DistanceUnit.INCH))/2));
        int[] returnval = {(int)x, (int)y};
        return  returnval;
    }
}

/*
switch(id)
        {
        case xback:
        returnval += interval*xspeedraw;
        break;
        case xfront:
        returnval += interval*xspeedraw;
        break;
        case yleft:
        returnval += interval*yspeedraw;
        break;
        case yright:
        returnval += interval*yspeedraw;
        break;
        }
        */