package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Mecanum", group = "Smart")

public class Test_Mecanum extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(team.red2, setupType.autonomous);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            /*alignUltrasonic();
            Pair loc = location();
            telemetry.addData("X: ", loc.x);
            telemetry.addData("Y: ", loc.y);
            telemetry.update();
            sleep(5000);
            break;
            */

        }
    }

}
