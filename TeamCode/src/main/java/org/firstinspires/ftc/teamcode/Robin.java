package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Central;


/**
 * Created by arulgupta on 12/15/17.
 */
@TeleOp(name="Tele-Op Robin", group = "Robin")

public class Robin extends Central {

    public static float yAxis1;
    public static float xAxis1;

    public static float yAxis2;
    public static float xAxis2;

    public static boolean motorRun;
    public static boolean motorRun2;


    public static float fb;
    public static float rl;

    public static double diagonalSpeed;

    public static boolean rightStickButtonPressed;
    public static boolean leftStickButtonPressed;
    //UNCOMMENT CRAWL MODE ACTIVATED TLEMETRY
    @Override
    public void runOpMode() throws InterruptedException {
        CentralClass(team.blue1, setupType.robin);


        waitForStart();

        while (opModeIsActive()) {
            yAxis1 = -gamepad1.left_stick_y; // Main Directions y-axis
            xAxis1 = gamepad1.left_stick_x;  // Main Directions x-axis

            yAxis2 = -gamepad1.right_stick_y; // Diagonal Directions y-axis
            xAxis2 = gamepad1.right_stick_x;  // Diagonal Directions x-axis

            fb = Math.abs(yAxis1);
            rl = Math.abs(xAxis1);

            rightStickButtonPressed = gamepad1.right_stick_button;  // Button Directions y-axis
            leftStickButtonPressed = gamepad1.left_stick_button;   // Button Directions x-axis

            // clip the right/left values so that the values never exceed +/- 1
            yAxis1 = Range.clip(yAxis1, -1, 1);
            xAxis1 = Range.clip(xAxis1, -1, 1);

            yAxis2 = Range.clip(yAxis2, -1, 1);
            xAxis2 = Range.clip(xAxis2, -1, 1);

            if (Math.pow(xAxis1, 2) + Math.pow(yAxis1, 2) >= Math.pow(DEAD_ZONE_SIZE, 2)) { //MAIN DIRECTIONS
                motorRun = true;
                if (yAxis1 >= Math.abs(2*xAxis1)) {
                    rightMotor.setPower(0.5);
                    leftMotor.setPower(-0.5);
                    telemetry.addLine("FORWARD");

                    //FORWARD
                } else if (yAxis1 <= -Math.abs(2*xAxis1)) {
                    telemetry.addLine("BACKWARD");
                    rightMotor.setPower(-0.5);
                    leftMotor.setPower(0.5);
                    //BACKWARD
                } else if (yAxis1 >= xAxis1/2 && yAxis1 < 2 * xAxis1) {
                    telemetry.addLine("FR");
                    rightMotor.setPower(0.2);
                    leftMotor.setPower(-0.7);

                    //Forward Right
                } else if (yAxis1 >= -xAxis1/2 && yAxis1 < 2 * -xAxis1) {
                    telemetry.addLine("FL");
                    rightMotor.setPower(0.7);
                    leftMotor.setPower(-0.2);
                    //Forward Left
                }
                else if (yAxis1 <= xAxis1/2 && yAxis1 > 2 * xAxis1) {
                    telemetry.addLine("BR");
                    rightMotor.setPower(-0.7);
                    leftMotor.setPower(0.2);

                    //Backward Left
                }
                else if (yAxis1 <= -xAxis1/2 && yAxis1 > 2 * -xAxis1) {
                    telemetry.addLine("BL");

                    rightMotor.setPower(-0.2);
                    leftMotor.setPower(0.7);
                    //Backward Right

                }
                else {
                    motorRun = false;
                    if (!motorRun2) {
                        stopDrivetrain();
                    }
                }
            }
            else if (Math.pow(xAxis2, 2) + Math.pow(yAxis2, 2) >= Math.pow(DEAD_ZONE_SIZE, 2)) {    //DIAGONAL
                if (yAxis2 >= Math.abs(xAxis2)) {
                    telemetry.addLine("LIFT UP");
                    lift.setPower(0.4);
                    //Up
                } else if (yAxis2 <= -Math.abs(xAxis2)) {
                    telemetry.addLine("LIFT DOWN");
                    lift.setPower(-0.4);
                    //Down
                }
                else {
                    lift.setPower(0);
                    telemetry.addLine("LIFT STOP");
                }

                if (Math.abs(yAxis2) < xAxis2 && !motorRun) {
                    motorRun2 = true;
                    telemetry.addLine("CW");
                    rightMotor.setPower(0.3);
                    leftMotor.setPower(0.3);
                    //CW

                } else if (-Math.abs(yAxis2) > xAxis2 && !motorRun) {
                    motorRun2 = true;
                    telemetry.addLine("CCW");
                    rightMotor.setPower(-0.3);
                    leftMotor.setPower(-0.3);
                    //CCW
                }
                else {
                    motorRun2 = false;
                    if (!motorRun) {
                        stopDrivetrain();
                    }
                }

            } else {
                motorRun = false;
                motorRun2 = false;
                telemetry.addLine("STOP");
                lift.setPower(0);
                stopDrivetrain();
            }

            motorRun = false;


            if (gamepad1.x){
                telemetry.addLine("Flick forward");
                flicker.setPower(-0.6);
                //encoderMovement(0.6, FLICKER_CONVERSION/2.5, 3, 20, movements.flick, flicker);

            }
            else if (gamepad1.b){
                telemetry.addLine("Flick backward");
                flicker.setPower(0.6);
            }
            else {
                telemetry.addLine("Flick STOP");
                flicker.setPower(0);
            }

            if (gamepad1.left_trigger > 0.25){
                telemetry.addLine("Open");
                rightServo.setPosition(rightServo.getPosition() + 0.05);
                leftServo.setPosition(leftServo.getPosition() - 0.05);

            }
            else if(gamepad1.right_trigger > 0.25){
                telemetry.addLine("Close");
                rightServo.setPosition(rightServo.getPosition() - 0.05);
                leftServo.setPosition(leftServo.getPosition() + 0.05);
            }
/*
            if (gamepad1.right_trigger > 0.25){
                telemetry.addLine("Close");
                rightServo.setPosition(rightServo.getPosition() + 0.05);
                leftServo.setPosition(leftServo.getPosition() + 0.05);

            }
            else if(gamepad1.left_trigger > 0.25){
                telemetry.addLine("Open");
                rightServo.setPosition(rightServo.getPosition() - 0.05);
                leftServo.setPosition(leftServo.getPosition() - 0.05);
            }

*/
            telemetry.update();

        }

    }

    @Override
    public void stopDrivetrain(){
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}
