package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/14/17.
 */
@Autonomous(name = "Auto-Tele-Op", group = "Smart")

public class AutoTeleOp extends Central {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);

        CentralClass(team.red2, setupType.autonomous);

        sleep(1000);


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            alignUltrasonic();
            Pair loc = location();
            telemetry.addData("X: ", loc.x);
            telemetry.addData("Y: ", loc.y);
            telemetry.update();



            break;

        }
    }

}
