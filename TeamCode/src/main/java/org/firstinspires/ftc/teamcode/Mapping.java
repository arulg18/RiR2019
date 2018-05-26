package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.NewStructure.NewCentral;

import java.io.IOException;

public class Mapping extends Central {

    public Field field;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void setField() throws IOException{
        field = new Field();

    }
    public void moveTo(Pair current, Pair destination){

    }
}
