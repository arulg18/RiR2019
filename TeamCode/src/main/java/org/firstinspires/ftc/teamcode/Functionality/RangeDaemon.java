package org.firstinspires.ftc.teamcode.Functionality;
/**
 * Created by Administrator on 04/24/2018.
 */

public class RangeDaemon extends RangePrediction implements Runnable {
    public void run()
    {
        while(opModeIsActive())//or whatever
        {
            RangePrediction.SmartPos();
            try{this.wait(interval);}
            catch(Exception e){}
        }
    }
}
