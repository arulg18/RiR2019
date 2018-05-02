package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Configuration.*;

import static org.firstinspires.ftc.teamcode.ServoValues.*;



/**
 * Created by arulgupta on 3/16/18.
 */

public class Central extends LinearOpMode {


    public ElapsedTime runtime = new ElapsedTime();

    //--------------------------------ENCODERS-------------------------
    private static final double COUNTS_PER_MOTOR_NEVEREST = 1680;
    private static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_NEVEREST * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
    public static final double COUNTS_PER_TETRIX_INCH = (COUNTS_PER_MOTOR_TETRIX * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);               // Conversion: Encoder Counts Motor Tetrix to Inches

    public static final double FLICKER_CONVERSION = 1680/(2*COUNTS_PER_INCH);
    //--------------------------------TELE-OP VALUES--------------------
    protected static final double ROTATION_SPEED = 0.8;
    protected static final double DEAD_ZONE_SIZE = 0.1;
    protected static final double D_PAD_SPEED = 0.4;
    protected static final double CRAWL_SPEED = 0.2;

    //-------------------------JEWEL SENSOR----------------------
    private static final int RED_COLOR_VALUE = 5;
    private static final int BLUE_COLOR_VALUE = 1;
    private static int count = 0;
    public static final boolean JEWEL_SENSOR_LED_ON = true;
    public static final long JEWEL_SENSOR_DETECT_PERIOD = 1500;

    //-----------------------ENCODER TRACKING---------------------
    public static Pair encStartLoc;
    public static float encStartAngle;

    //-----------------------ULTRASONIC---------------------
    public static double ubl;
    public static double ubr;
    public static double ull;
    public static double ulr;

    public static final double snapSpeed = 0.05;
    public static final double xCenterOffset = 4; //VALUE TO BE CHANGED
    public static final double yCenterOffset = 4; //VALUE TO BE CHANGED

    public static final double xDistBetweenUltra = 5;
    public static final double yDistBetweenUltra = 5;
    public static final double robotLength = 18;

    //-----------------------RangeFinders------------------------------
    public  static ModernRoboticsI2cRangeSensor yleft;
    public  static ModernRoboticsI2cRangeSensor yright;
    public  static ModernRoboticsI2cRangeSensor xfront;
    public  static ModernRoboticsI2cRangeSensor xback;


    //--------------------------ENUMERATIONS---------------------
    public enum movements{
        backward(-1, 1, -1, 1),
        forward(1, -1, 1, -1),
        left(1, 1, -1, -1),
        right(-1, -1, 1, 1),
        tr(0, -1, 1, 0),
        tl(1, 0, 0, -1),
        br(-1, 0, 0, 1),
        bl(0, 1, -1, 0),
        ccw(-1, -1, -1, -1),
        cw(1, 1, 1, 1),
        cwback(-1,-1,0,0),
        ccwback(1,1,0,0),
        cwfront(0,0,-1,-1),
        ccwfront(0,0,1,1),
        flick(-1),
        glyphUp,
        glyphDown,
        treadUp,
        treadDown,
        relicOut(-1),
        relicIn(1);

        private final double[] directions;

        movements(double... signs){
            this.directions = signs;
        }

        private double[] getDirections(){
            return directions;
        }
    }

    public enum setupType{
        autonomous, glyph, robin, jewel, relic, drive, teleop;
    }

    public enum team{
        red1, red2, blue1, blue2;
    }

    public enum flick{
        right, left
    }

    public enum EncoderMode{
        ON, OFF
    }
    public enum cryptoboxSide{
        left, center, right
    }

        //---------- IMU ENUMERATIONS -----------
            public enum turnside{
                ccw, cw
            }
            public enum axis{
                front, center, back
            }


    //--------------------------CONFIGURATIONS-----------------------
    public static BNO055IMUImpl imu;
    public BNO055IMUImpl.Parameters parameters = new BNO055IMUImpl.Parameters();
    public Orientation current;
    float initorient;
    float start;
    float end;
    float xtilt;
    float ytilt;
    public static final double sensitivity = 1;
    public static boolean isnotstopped;
    public team thisteam;

    public static HardwareMap thishardwareMap;
    public static float perpZ;


    //  Drivetrain
    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;

    //  Jewel Systems
    public Servo jewelDown;
    public Servo jewelFlick;

    public ColorSensor jewelSensor;
    //

    //  Glyph System
    public Servo pullServo;
    public DcMotor rightTread;
    public DcMotor leftTread;
    public ColorSensor glyphColorSensor;

    //  Relic Systems
    public DcMotor relicMotorIn;
    public Servo Claw;
    public Servo angleServo;


    // AutoGlyph
    public Servo RGrabber;
    public Servo LGrabber;

    //AutoTeleOp
    public ModernRoboticsI2cRangeSensor backLeft   ;
    public ModernRoboticsI2cRangeSensor backRight  ;
    public ModernRoboticsI2cRangeSensor leftLeft   ;
    public ModernRoboticsI2cRangeSensor leftRight  ;


    //ROBIN
    public DcMotor rightMotor;
    public DcMotor leftMotor;

    public DcMotor lift;
    public DcMotor flicker;

    public Servo rightServo;
    public Servo leftServo;


    // ARRAYS OF SYSTEMS

    public DcMotor[] drivetrain = new DcMotor[4];
    public DcMotor[] glyphSystem = new DcMotor[3];

    public Servo[] relicSystem = new Servo[2];
    public Servo[] jewelSystem = new Servo[2];

    // DIRECTIONS
    public movements[] allMovements = {movements.forward, movements.backward, movements.right, movements.left, movements.tr, movements.tl, movements.br, movements.bl, movements.cw, movements.ccw, movements.cwback, movements.ccwback, movements.ccwfront, movements.cwfront};

    //----------------------------WAIT TIMES-----------------------------
    public static final int CENTER_FLICKER_WAIT = 200;
    public static final int INIT_FLICKER_WAIT = 200;
    public static final int FLICK_END_PAUSE = 1000;



    //----------------------------------------------------------------------------------------------------------------------------------------------------------

    public Central(){
    }

    public void CentralClass(team player, setupType... setup) throws InterruptedException{
        thishardwareMap = hardwareMap;
        thisteam = player;
        for (setupType type : setup) {
            switch (type){
                case autonomous:
                    setupAutoTeleOp();
                    break;
                case teleop:

                    break;
                case drive:
                    setupDrivetrain();
                    break;
                case jewel:
                    setupJewel();
                    break;
                case relic:
                    setupRelic();
                    break;
                case glyph:
                    setupGlyph();
                    break;
                case robin:
                    setupRobin();
                    break;
            }
        }

    }

    public void runOpMode() throws InterruptedException {
        CentralClass(team.blue1, setupType.autonomous);
    }

    //------------------------ENCODER MOVEMENTS----------------------------

    public void driveTrainEncoderMovement(double speed, double distance, double timeoutS, long waitAfter, Central.movements movement) throws  InterruptedException{

        int[] targets = new int[drivetrain.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:drivetrain){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: drivetrain){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: drivetrain){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }
    protected void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, Central.movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getDirections();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }


    //------------------JEWEL FUNCTIONS------------------------------------------------------------------------
    protected void centerFlicker(long waitAfter) throws InterruptedException{
        jewelDown.setPosition(CENTER_POSITION_DOWN);
        jewelFlick.setPosition(CENTER_POSITION_FLICK);
        sleep(CENTER_FLICKER_WAIT + waitAfter);
    }
    protected void initialPositionFlicker(long waitAfter) throws InterruptedException{
        jewelDown.setPosition(START_POSITION_DOWN);
        jewelFlick.setPosition(START_POSITION_FLICK);
        sleep(INIT_FLICKER_WAIT + waitAfter);
    }
    protected void sweepServo(Servo servo, double endPosition, double increment, long incrementSpeed) throws InterruptedException{
        if (servo.getPosition() > endPosition){
            for (double p = servo.getPosition(); servo.getPosition() > endPosition; p-= increment){
                if (!opModeIsActive()){
                    break;
                }
                servo.setPosition(p);
                sleep(incrementSpeed);
            }
        }else {
            for (double p = servo.getPosition(); servo.getPosition() < endPosition; p+= increment){
                if (!opModeIsActive()){
                    break;
                }
                servo.setPosition(p);
                sleep(incrementSpeed);
            }
        }
    }

    protected void Red() throws InterruptedException{
        if (jewelSensor.red() >= RED_COLOR_VALUE) { //FLICK REG
            flick(flick.left);
            telemetry.addLine("Red");
            telemetry.update();

        } else if (jewelSensor.blue() >= BLUE_COLOR_VALUE) {                               //FLICK OPPOSITE
            flick(flick.right);
            telemetry.addLine("Blue");
            telemetry.update();
        }
        else{
            jewelFlick.setPosition(jewelFlick.getPosition() - .01);
            if (count < 7){
                count++;

                Red();
            }

        }
    }
    protected void Blue() throws InterruptedException{
        if (jewelSensor.red() >= RED_COLOR_VALUE) { //FLICK REG
            flick(flick.right);
            telemetry.addLine("Red");
            telemetry.update();

        } else if (jewelSensor.blue() >= BLUE_COLOR_VALUE) {                               //FLICK OPPOSITE
            flick(flick.left);
            telemetry.addLine("Blue");
            telemetry.update();
        }
        else {
            jewelFlick.setPosition(jewelFlick.getPosition() - .01);

            if (count < 5){
                count++;
                Blue();
            }
        }
    }
    public void flick(team side) throws InterruptedException{
        centerFlicker(0);
        sweepServo(jewelDown, LOW_POSITION_DOWN, INCREMENT_POSITION_DOWN, INCREMENT_FREQUENCY_DOWN);

        jewelSensor.enableLed(JEWEL_SENSOR_LED_ON);

        sleep(JEWEL_SENSOR_DETECT_PERIOD);
        telemetry.addData("Blue Value: ", jewelSensor.blue());
        telemetry.update();
        switch (side){
            case red1:
            case red2:

                Red();
                break;
            case blue1:
            case blue2:

                Blue();
                break;
        }
        sleep(FLICK_END_PAUSE);
        centerFlicker(0);
    }
    public void flick(flick side) throws InterruptedException{
        switch (side){
            case left:
                jewelFlick.setPosition(LEFT_POSITION_FLICK);
                telemetry.addData("Position:", jewelFlick.getPosition());
                telemetry.addLine("Left");
                telemetry.update();

                break;
            case right:
                jewelFlick.setPosition(RIGHT_POSITION_FLICK);
                telemetry.addData("Position:", jewelFlick.getPosition());
                telemetry.addLine("Right");

                telemetry.update();
                break;
        }
    }


    //--------------------MOVEMENT FUNCTIONS-------------------------------------
    protected void turn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException{
        start = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        end = start + ((direction == turnside.cw) ? target : -target);
        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (java.lang.InterruptedException e) {
            isnotstopped = false;
        }
        while (!((end <= current.firstAngle + 1) && end > current.firstAngle - 1) && opModeIsActive() && isnotstopped) {
            current = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        try {
            stopDrivetrain();
        } catch (java.lang.InterruptedException e) {
        }

    }
    protected void turn(float target, turnside direction, double speed) throws InterruptedException {
        turn(target, direction, speed, axis.center);
    }
    public void turn(float target, turnside direction) throws InterruptedException {
        turn(target, direction, 10);
    }
    protected void absturn(float target, turnside direction, double speed, axis rotation_Axis) throws InterruptedException { //very similar to turn(target, direction, speed, rotation_Axis), fix if messy
        float turnval = (target + initorient + 180) % 360 - 180;
        isnotstopped = true;
        try {
            switch (rotation_Axis) {
                case center:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cw : movements.ccw);
                    break;
                case back:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwback : movements.ccwback);
                    break;
                case front:
                    driveTrainMovement(speed, (direction == turnside.cw) ? movements.cwfront : movements.ccwfront);
                    break;
            }
        } catch (java.lang.InterruptedException e) {
            isnotstopped = false;
        }
        while (!((turnval <= current.firstAngle + 1) && turnval > current.firstAngle - 1) && opModeIsActive() && isnotstopped) {
            current = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        try {
            stopDrivetrain();
        } catch (java.lang.InterruptedException e) {
        }
    }
    protected void tipcorrect() throws InterruptedException {
        xtilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        ytilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        double angleoff = Math.pow((Math.sin(Math.toRadians((double) xtilt))), 2) + Math.pow(Math.sin(Math.toRadians((double) (ytilt))), 2);
        if (angleoff > Math.sin(Math.toRadians(sensitivity))) {
            if ((xtilt > 0 && ytilt > 0 && ytilt < xtilt) || (xtilt > 0 && ytilt < 0 && xtilt > -ytilt))//right
            {
                movetry(movements.right);
            } else if ((xtilt > 0 && ytilt > 0 && xtilt < ytilt) || (xtilt < 0 && ytilt > 0 && -xtilt < ytilt))//forwards
            {
                movetry(movements.forward);
            } else if ((xtilt < 0 && ytilt > 0 && -xtilt > ytilt) || (xtilt < 0 && ytilt < 0 && -xtilt > -ytilt))//left
            {
                movetry(movements.left);
            } else if ((xtilt < 0 && ytilt < 0 && -xtilt > -ytilt) || (xtilt > 0 && ytilt < 0 && xtilt < -ytilt))//back
            {
                movetry(movements.backward);
            }
            try {
                tipcorrect();
            } catch (java.lang.InterruptedException e) {
                try {
                    stopDrivetrain();
                } catch (java.lang.InterruptedException i) {
                }
            }
        } else {
            try {
                stopDrivetrain();
            } catch (java.lang.InterruptedException i) {
            }
        }
    }
    protected boolean movetry(movements direction) throws InterruptedException{
        try {
            driveTrainMovement(0.2, direction);
        } catch (java.lang.InterruptedException e) {
            try {
                stopDrivetrain();
                return false;
            } catch (java.lang.InterruptedException i) {
            }
        }
        return true;
    }
    protected void balancer() throws InterruptedException{ // very similar to tipcorrect(), fix if messy
        xtilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle ;
        ytilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle ;
        double angleoff = Math.pow((Math.sin(Math.toRadians((double) xtilt))), 2) + Math.pow(Math.sin(Math.toRadians((double) (ytilt))), 2);
        telemetry.addData("Angle Offset: ", Math.toDegrees(angleoff));

        telemetry.addData("X-tilt: ", xtilt);
        telemetry.addData("Y-tilt: ", ytilt);
        if (Math.abs(xtilt) > sensitivity || Math.abs(ytilt) > sensitivity) {
            telemetry.addData("angleoff > Math.sin(Math.toRadians(sensitivity))", true);

            if (Math.abs(ytilt) < xtilt)//left
            {
                telemetry.addData("left", true);
                driveTrainMovement(0.2, movements.right);
            } else if (ytilt >= Math.abs(xtilt))//backwards
            {
                telemetry.addData("backwards", true);
                driveTrainMovement(0.2, movements.forward);

            } else if (-Math.abs(ytilt) > xtilt)//right
            {
                driveTrainMovement(0.2, movements.left);
                telemetry.addData("right", true);
            } else if (ytilt <= -Math.abs(xtilt))//forwards
            {
                driveTrainMovement(0.2, movements.backward);
                telemetry.addData("forward", true);
            }
            else {
                telemetry.addLine("No direction");
            }
            telemetry.update();

            balancer();

        } else {
            telemetry.addData("angleoff > Math.sin(Math.toRadians(sensitivity))", false);
            telemetry.update();

            stopDrivetrain();

        }
    }
    protected void balancer(float startX, float startY) throws InterruptedException{ // very similar to tipcorrect(), fix if messy
        xtilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle - startX;
        ytilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle - startY;
        double angleoff = Math.pow((Math.sin(Math.toRadians((double) xtilt))), 2) + Math.pow(Math.sin(Math.toRadians((double) (ytilt))), 2);
        telemetry.addData("Angle Offset: ", Math.toDegrees(angleoff));

        telemetry.addData("X-tilt: ", xtilt);
        telemetry.addData("Y-tilt: ", ytilt);
        if (Math.abs(xtilt) > sensitivity || Math.abs(ytilt) > sensitivity) {
            telemetry.addData("angleoff > Math.sin(Math.toRadians(sensitivity))", true);

            if (Math.abs(ytilt) < xtilt)//left
            {
                telemetry.addData("left", true);
                driveTrainMovement(0.2, movements.right);
            } else if (ytilt >= Math.abs(xtilt))//backwards
            {
                telemetry.addData("backwards", true);
                driveTrainMovement(0.2, movements.forward);

            } else if (-Math.abs(ytilt) > xtilt)//right
            {
                driveTrainMovement(0.2, movements.left);
                telemetry.addData("right", true);
            } else if (ytilt <= -Math.abs(xtilt))//forwards
            {
                driveTrainMovement(0.2, movements.backward);
                telemetry.addData("forward", true);
            }
            else {
                telemetry.addLine("No direction");
            }
            telemetry.update();

            balancer(startX, startY);

        } else {
            telemetry.addData("angleoff > Math.sin(Math.toRadians(sensitivity))", false);
            telemetry.update();

            stopDrivetrain();

        }
    }
    protected void balancer(float startX, float startY, Gamepad gamepad, double speed) throws InterruptedException{ // very similar to tipcorrect(), fix if messy
        if (gamepad.b){
            telemetry.addLine("Balance mode OFF");
            telemetry.update();
        }
        else {
            telemetry.addLine("Balance mode ON");
            telemetry.update();
            xtilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle - startX;
            ytilt = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle - startY;
            double angleoff = Math.pow((Math.sin(Math.toRadians((double) xtilt))), 2) + Math.pow(Math.sin(Math.toRadians((double) (ytilt))), 2);
            telemetry.addData("Angle Offset: ", Math.toDegrees(angleoff));

            telemetry.addData("X-tilt: ", xtilt);
            telemetry.addData("Y-tilt: ", ytilt);
            if (Math.abs(xtilt) > sensitivity || Math.abs(ytilt) > sensitivity) {
                telemetry.addData("angleoff > Math.sin(Math.toRadians(sensitivity))", true);

                if (Math.abs(ytilt) < xtilt)//left
                {
                    telemetry.addData("left", true);
                    driveTrainMovement(0.2, movements.right);
                } else if (ytilt >= Math.abs(xtilt))//backwards
                {
                    telemetry.addData("backwards", true);
                    driveTrainMovement(0.2, movements.forward);

                } else if (-Math.abs(ytilt) > xtilt)//right
                {
                    driveTrainMovement(0.2, movements.left);
                    telemetry.addData("right", true);
                } else if (ytilt <= -Math.abs(xtilt))//forwards
                {
                    driveTrainMovement(0.2, movements.backward);
                    telemetry.addData("forward", true);
                }
                else {
                    telemetry.addLine("No direction");
                }
                telemetry.update();

                balancer(startX, startY, gamepad, speed);

            } else {
                telemetry.addData("angleoff > Math.sin(Math.toRadians(sensitivity))", false);
                telemetry.update();

                stopDrivetrain();

            }
        }
    }


    //------------------SET FUNCTIONS------------------------------------------------------------------------
    public void setRuntime(ElapsedTime time) throws InterruptedException {
        runtime = time;
    }

    //------------------RELIC FUNCTIONS------------------------------------------------------------------------
    //none right now
    //------------------AUTO TELE-OP FUNCTIONS------------------------------------------------------------------------
    public void alignUltrasonic() throws InterruptedException{
        ubl = backLeft.getDistance(DistanceUnit.INCH);
        ubr = backRight.getDistance(DistanceUnit.INCH);
        ull = leftLeft.getDistance(DistanceUnit.INCH);
        ulr = leftRight.getDistance(DistanceUnit.INCH);
        int a = isPerpendicular();
        if (a < 0){ // left is smaller, turn ccw
            snapUltra(turnside.ccw);
        }
        else if (a > 0){ // right is smaller, turn cw
            snapUltra(turnside.cw);
        }
        setIMUPerpendicular();
    }


    public void snapUltra(turnside direction) throws InterruptedException{
        switch (direction){
            case cw:
                driveTrainMovement(snapSpeed, movements.cw);
                while (opModeIsActive() && ull != ulr && ubl != ubr){
                     // function may be spinning wrong
                }
                stopDrivetrain();
                break;
            case ccw:
                driveTrainMovement(snapSpeed, movements.ccw);
                while (opModeIsActive() && ull != ulr && ubl != ubr){
                    // function may be spinning wrong
                }
                stopDrivetrain();
                break;
        }
    }
    public void setIMUPerpendicular(){
        perpZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }
    public float getIMUZ(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle - perpZ;

    }
    public Pair location(){
        Pair current;
        ubl = backLeft.getDistance(DistanceUnit.INCH);
        ubr = backRight.getDistance(DistanceUnit.INCH);
        ull = leftLeft.getDistance(DistanceUnit.INCH);
        ulr = leftRight.getDistance(DistanceUnit.INCH);
        if (isPerpendicular() == 0) {
            current = convertToCenter((ubl + ubr) / 2, (ull + ulr) / 2);
        }
        else {
            double xDiff = ull - ulr;
            double yDiff = ubl - ubr;
            double thetax1 = Math.atan2(xDiff, xDistBetweenUltra);
            double thetay1 = Math.atan2(yDiff, yDistBetweenUltra);
            double theta = (thetax1 + thetay1) / 2;

            double offsetHeight = robotLength * Math.sin(theta);

            double toEdgeY = (ubl - (robotLength - yDistBetweenUltra) * Math.tan(theta) / 2) * Math.sin(Math.PI - theta);
            double y = toEdgeY + offsetHeight / 2;

            double toEdgeX = (ull - (robotLength - xDistBetweenUltra) * Math.tan(theta) / 2) * Math.sin(Math.PI - theta);
            double x = toEdgeX + offsetHeight / 2;

            current = convertToCenter(x, y, (float) theta);
        }
        return current;

    }
    public int isPerpendicular(){
        if (ubl < ubr || ull < ulr){
            return -1;                  // left is smaller, turn ccw
        }
        else if (ubl > ubr || ull > ulr){
            return  1;                  // right is smaller, turn cw
        }
        else {
            return 0;                   // perpendicular
        }

    }

    public Pair convertToCenter(double x, double y){
        return new Pair(x + xCenterOffset, y + yCenterOffset, 0);
    }
    public Pair convertToCenter(double x, double y, float orient){
        return new Pair(x + xCenterOffset, y + yCenterOffset, orient);
    }
    public void encoderTrackingOn(){
        encStartLoc = location();
        encStartAngle = getIMUZ();


    }
    public void addToEncoder(movements movement){
        int[] xy = new int[2];
        int rotation;

    }

    public void move(Pair destination, float orientation){ // destination is not rotated
        Pair current = location();
        while (current.getX() != destination.getX() && current.getY() != destination.getY()){
            // MOVE ALGORITHM


        }


    }

    //------------------GLYPH FUNCTIONS------------------------------------------------------------------------
        protected void GlyphDown() throws InterruptedException{
            pullServo.setPosition(HIGH_POSITION_PULL);
            sleep(500);
            powerMotors(-1, 2000, rightTread, leftTread);
        }
        protected void GlyphDownONALL() throws InterruptedException{
            pullServo.setPosition(HIGH_POSITION_PULL);
            sleep(500);

            rightTread.setPower(-1);
            leftTread.setPower(-1);
        }
        protected void GlyphOFF() throws InterruptedException{
            pullServo.setPosition(0.8);
            rightTread.setPower(0);
            leftTread.setPower(0);
            sleep(500);
        }


    //------------------AUTOGLYPH FUNCTIONS------------------------------------------------------------------------
    protected void autoGrabGlyph() throws InterruptedException{
        RGrabber.setPosition(RGRAB_POSITION);
        LGrabber.setPosition(LGRAB_POSITION);
    }
    protected void autoDropGlyph() throws InterruptedException{
        RGrabber.setPosition(R_READY_POSITION);
        LGrabber.setPosition(L_READY_POSITION);
    }
    protected void autoReturnServo() throws InterruptedException{
        RGrabber.setPosition(START_POSITION_RGRAB);
        LGrabber.setPosition(START_POSITION_LGRAB);
    }

    //------------------HARDWARE SETUP FUNCTIONS------------------------------------------------------------------------
    public DcMotor motor(String name, DcMotor.Direction direction) throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }
    public void motorDriveMode(EncoderMode mode, DcMotor... motor) throws InterruptedException {
        switch (mode) {
            case ON:
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                idle();
                for (DcMotor i : motor) {
                    i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case OFF:
                break;
        }

        this.drivetrain = motor;

    }
    public Servo servo(String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException {
        Servo servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public CRServo servo(String name, DcMotorSimple.Direction direction, double startSpeed) throws InterruptedException {
        CRServo servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);

        servo.setPower(0);
        return servo;
    }
    public ColorSensor colorSensor(String name, boolean ledOn) throws InterruptedException {
        ColorSensor sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        telemetry.addData("Beacon Red Value: ", sensor.red());
        telemetry.update();

        return sensor;
    }
    public ModernRoboticsI2cRangeSensor ultrasonicSensor(String name) throws InterruptedException {

        return hardwareMap.get(ModernRoboticsI2cRangeSensor.class, name);
    }

    public void powerMotors(double speed, long time, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
        sleep(time);
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }



    public void setupDrivetrain() throws InterruptedException {
        motorFR = motor(motorFRS, DcMotorSimple.Direction.FORWARD);
        motorFL = motor(motorFLS, DcMotorSimple.Direction.FORWARD);
        motorBR = motor(motorBRS, DcMotorSimple.Direction.FORWARD);
        motorBL = motor(motorBLS, DcMotorSimple.Direction.FORWARD);

        motorDriveMode(EncoderMode.ON, motorFR, motorFL, motorBR, motorBL);
    }
    public void setupRelic() throws InterruptedException{
        angleServo = servo(angleServoS, Servo.Direction.FORWARD, MIN_POSITION_WRIST, MAX_POSITION_WRIST, START_POSITION_WRIST);
        Claw = servo(ClawS, Servo.Direction.FORWARD, MIN_POSITION_CLAW, MAX_POSITION_CLAW, START_POSITION_CLAW);
        relicMotorIn = motor(relicMotorInS, DcMotorSimple.Direction.FORWARD);
        telemetry.addLine("Setup");
        telemetry.update();
    }// FINISH
    public void setupJewel() throws InterruptedException{
        jewelDown = servo(jewelDownS, Servo.Direction.FORWARD, MIN_POSITION_DOWN, MAX_POSITION_DOWN, START_POSITION_DOWN);
        jewelFlick = servo(jewelFlickS, Servo.Direction.FORWARD, MIN_POSITION_FLICK, MAX_POSITION_FLICK, START_POSITION_FLICK);

        jewelSensor = colorSensor(jewelSensorS, JEWEL_SENSOR_LED_ON);

    }
    public void setupGlyph() throws InterruptedException{
        pullServo = servo(pullServoS, Servo.Direction.FORWARD, MIN_POSITION_PULL, MAX_POSITION_PULL, START_POSITION_PULL);
        leftTread = motor(leftTreadS, DcMotorSimple.Direction.FORWARD);
        rightTread = motor(rightTreadS, DcMotorSimple.Direction.FORWARD);
    }
    public void setupAutoGlyph() throws InterruptedException{
        telemetry.addLine("starting...");
        telemetry.update();
        LGrabber = servo(LGrabberS, Servo.Direction.FORWARD, MIN_POSITION_LGRAB, MAX_POSITION_LGRAB, START_POSITION_LGRAB);
        RGrabber = servo(RGrabberS, Servo.Direction.FORWARD, MIN_POSITION_RGRAB, MAX_POSITION_RGRAB, R_READY_POSITION);
        sleep(1000);
        RGrabber.setPosition(START_POSITION_RGRAB);

        telemetry.addLine("Done.");
        telemetry.update();

    }
    public void setupIMU(team side) throws InterruptedException {
        parameters.angleUnit = BNO055IMUImpl.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMUImpl.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true; //copypasted from BNO055IMU sample code, no clue what this does
        parameters.loggingTag = "IMU"; //copypasted from BNO055IMU sample code, no clue what this does
        imu = thishardwareMap.get(BNO055IMUImpl.class, imuRedS);
        imu.initialize(parameters);
        Position startpos;
        /*switch (side) { //initialize the position with correct coordinates
            case red1:
                startpos = new Position(DistanceUnit.INCH, RedX, OneY, 0, 0);
                AngleOffset = 90;
                break;
            case red2:
                startpos = new Position(DistanceUnit.INCH, RedX, TwoY, 0, 0);
                AngleOffset = 90;
                break;
            case blue1:
                startpos = new Position(DistanceUnit.INCH, BlueX, OneY, 0, 0);
                AngleOffset = -90;
                break;
            case blue2:
                startpos = new Position(DistanceUnit.INCH, BlueX, TwoY, 0, 0);
                AngleOffset = -90;
                break;
            default:// never happens, but needed to compile
                startpos = new Position();
                break;
        }*/
        //origin is @ bottom left when looking at the board with red1 @ top left corner
        // 0 degrees is @ east when looking at the board with red1 @ top left corner
        Velocity veloInit = new Velocity(DistanceUnit.INCH, 0, 0, 0, 0);
        imu.startAccelerationIntegration(new Position(),new Velocity(),20);
        initorient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void setupAutoTeleOp() throws InterruptedException{
        backLeft = ultrasonicSensor(backLeftS);
        backRight = ultrasonicSensor(backRightS);
        leftLeft = ultrasonicSensor(leftLeftS);
        leftRight = ultrasonicSensor(leftRightS);
    }
    public void setupRobin() throws InterruptedException{
        rightMotor = motor(rightMotorS, DcMotorSimple.Direction.FORWARD);
        leftMotor = motor(leftMotorS, DcMotorSimple.Direction.FORWARD);

        lift = motor(liftS, DcMotorSimple.Direction.FORWARD);
        flicker = motor(flickerS, DcMotorSimple.Direction.FORWARD);

        rightServo = servo(rightServoS, Servo.Direction.FORWARD, 0, 1, 0.5);
        leftServo = servo(leftServoS, Servo.Direction.FORWARD, 0, 1, 0.5);

        motorDriveMode(EncoderMode.ON, lift, flicker);
    }

    //------------------DRIVETRAIN TELEOP FUNCTIONS------------------------------------------------------------------------
    public void driveTrainMovement(double speed, Central.movements movement) throws InterruptedException{
        double[] signs = movement.getDirections();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x]* speed);

        }
    }

    public void driveTrainMovementAccelerate(double speed, Central.movements movement) throws InterruptedException{     // DOESNT WORK
        double[] signs = movement.getDirections();
        for (double i = 0; i <= speed; i+=.1) {
            for (DcMotor motor : drivetrain) {
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setPower(signs[x] * i);

            }
        }
    }

    public void stopDrivetrain() throws InterruptedException{ //why does this throw interrupted lol
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }

    static class Pair{
        double x, y;
        float orient;

        public Pair(double x, double y, float orient){
            this.x = x;
            this.y = y;
            this.orient = orient;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public float getOrient() {
            return orient;
        }
    }











}
