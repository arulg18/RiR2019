package org.firstinspires.ftc.teamcode.SimpleStructure;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AutonomousTest extends AutonomousCentral {

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        super.setRuntime(runtime);

        setup(team.none, setupType.none);
        sleep(1000);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){


            break;
        }
    }
}
