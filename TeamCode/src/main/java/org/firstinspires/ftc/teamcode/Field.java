package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class Field {

    public int field[][] = new int[145][145];

    public Field() throws IOException{
        BufferedReader f = new BufferedReader(new FileReader("Mapping.csv"));

        for (int i = 0; i < 145; i++) {
            String[] line = f.readLine().split(",");
            for (int j = 0; j < 145; j++) {
                field[i][j] = Integer.parseInt(line[j]);
            }
        }
    }
}
