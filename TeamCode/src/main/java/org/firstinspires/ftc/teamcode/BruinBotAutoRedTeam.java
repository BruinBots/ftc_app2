package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * Created by rohan on 11/5/2017.
 */

@Autonomous(name="BruinBotAutoRedTeam", group = "Vince")

public class BruinBotAutoRedTeam extends LinearOpMode {
    private Servo leftServo;
    private Servo rightServo;
    private Servo sensorServo;
    private DcMotor forkLift;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    ColorSensor colorSensor;
    double left;
    double right;
    double drive;
    double turn;
    double max;
    double liftUp;
    double liftDown;
    public int orderedPosition = 0;
    public static final double MID_SERVO = 0.5;


    @Override
    public void runOpMode() {
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
        forkLift = hardwareMap.get(DcMotor.class, "forkLift");
        sensorServo = hardwareMap.get(Servo.class, "sensorServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        //This section resets the forkLift... The forkLift MUST start in lowest position!
        /*forkLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Resets encoders
        while (forkLift.getCurrentPosition() != 0) { //Ensures encoders are zero
            forkLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.update(); //Needed within all loops
        }
        forkLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets mode to use encoders setMode() is used instead of setChannelMode(), which is now deprecated
        forkLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
*/
        //forkLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forkLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // Initialize the servos to the mid position
        leftServo.setPosition(MID_SERVO);
        rightServo.setPosition(MID_SERVO);
        sensorServo.setPosition(.05);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            //Extend arm on servo so it is between the balls
            sensorServo.setPosition(.6);

            ///Read the color sensor
// convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            //If our team color is sensed, turn the robot a little away, else turn the robot a little towards the sensor
            if (colorSensor.red()>colorSensor.blue()) {
                // Ball is Red, Turn Right
                leftWheel.setPower(.1);}
            else
            {// Ball is Blue, Turn Left
                rightWheel.setPower(.1);
            }
            //Put in Delay?
            leftWheel.setPower(0);
            rightWheel.setPower(0);

            //Retract the arm on servo
            sensorServo.setPosition(0);
            //Drive towards safety zone (number of wheel turns & distance), turn 90 degrees, drive into cryptobox
            rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightWheel.setTargetPosition(2880);
            leftWheel.setPower(.2);
            rightWheel.setPower((.2));


// ************* refresh data *********************** //

            telemetry.addData("Servo Position", rightServo.getPosition());
            telemetry.addData("Servo Position", leftServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();


        }


    }
}