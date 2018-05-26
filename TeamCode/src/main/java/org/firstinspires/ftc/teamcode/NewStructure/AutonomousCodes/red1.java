package org.firstinspires.ftc.teamcode.NewStructure.AutonomousCodes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NewStructure.Autonomous;
import org.firstinspires.ftc.teamcode.NewStructure.Robot;

public class red1 extends Autonomous{

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        setRuntime(runtime);
        robot = new Robot(hardwareMap);
        setTeam(team.red1);
        sleep(1000);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){

        }



    }
}
