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
            gamepad1.setJoystickDeadzone((float) 0.1);

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

            if (!leftStickButtonPressed) { //MAIN DIRECTIONS

                if (yAxis1 >= Math.abs(2*xAxis1)) {
                    rightMotor.setPower(0.5);
                    leftMotor.setPower(0.5);
                    telemetry.addLine("FORWARD");

                    //FORWARD
                } else if (yAxis1 <= -Math.abs(2*xAxis1)) {
                    telemetry.addLine("BACKWARD");
                    rightMotor.setPower(-0.5);
                    leftMotor.setPower(-0.5);
                    //BACKWARD
                } else if (yAxis1 >= xAxis1/2 && yAxis1 < 2 * xAxis1) {
                    telemetry.addLine("FR");
                    rightMotor.setPower(0.2);
                    leftMotor.setPower(0.5);

                    //Forward Right
                } else if (yAxis1 >= -xAxis1/2 && yAxis1 < 2 * -xAxis1) {
                    telemetry.addLine("FL");
                    rightMotor.setPower(0.5);
                    leftMotor.setPower(0.2);
                    //Forward Left
                }
                else if (yAxis1 <= xAxis1/2 && yAxis1 > 2 * xAxis1) {
                    telemetry.addLine("BR");

                    rightMotor.setPower(-0.2);
                    leftMotor.setPower(-0.5);
                    //Backward Right
                }
                else if (yAxis1 <= -xAxis1/2 && yAxis1 > 2 * -xAxis1) {
                    telemetry.addLine("BL");
                    rightMotor.setPower(-0.5);
                    leftMotor.setPower(-0.2);

                    //Backward Left
                }
                else {
                    telemetry.addLine("STOP");
                    stopDrivetrain();
                }
            } else if (!rightStickButtonPressed) {    //DIAGONAL
                if (yAxis1 >= Math.abs(xAxis2)) {
                    telemetry.addLine("LIFT UP");
                    lift.setPower(0.4);
                    //Up
                } else if (yAxis1 <= -Math.abs(xAxis2)) {
                    telemetry.addLine("LIFT DOWN");
                    lift.setPower(-0.4);
                    //Down
                } else if (Math.abs(yAxis2) < xAxis2) {
                    telemetry.addLine("CW");
                    rightMotor.setPower(-0.3);
                    leftMotor.setPower(0.3);
                    //CW

                } else if (-Math.abs(yAxis2) > xAxis2) {
                    telemetry.addLine("CCW");
                    rightMotor.setPower(0.3);
                    leftMotor.setPower(-0.3);
                    //CCW
                }
                else {
                    telemetry.addLine("STOP");
                    stopDrivetrain();
                }
            } else {
                telemetry.addLine("STOP");
                stopDrivetrain();
            }


            if (gamepad1.x){
                telemetry.addLine("Flick forward");
                flicker.setPower(0.6);

            }
            else if (gamepad1.b){
                telemetry.addLine("Flick backward");
                flicker.setPower(-0.6);
            }
            else {
                telemetry.addLine("Flick STOP");
                flicker.setPower(0);
            }

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

            telemetry.update();

        }

    }

}
