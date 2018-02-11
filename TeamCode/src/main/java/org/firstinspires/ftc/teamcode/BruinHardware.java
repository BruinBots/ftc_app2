package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;



/**
 * Created by rohan on 1/30/2018.
 */

public class BruinHardware {
    public DcMotor leftWheel = null;
    public DcMotor rightWheel = null;
    public DcMotor forkLift = null;
    public Servo leftServo = null;
    public Servo leftServo2 = null;
    public Servo rightServo = null;
    public Servo rightServo2 = null;
    public Servo sensorServo = null;
    public ColorSensor colorSensor = null;

    static double MID_SERVO = 0.5;


    HardwareMap hwMap = null;

    public BruinHardware() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftWheel = hwMap.get(DcMotor.class, "leftWheel");
        rightWheel = hwMap.get(DcMotor.class, "rightWheel");
        forkLift = hwMap.get(DcMotor.class, "forkLift");
        leftServo = hwMap.get(Servo.class, "leftServo");
        leftServo2 = hwMap.get(Servo.class, "leftServo2");
        rightServo = hwMap.get(Servo.class, "rightServo");
        rightServo2 = hwMap.get(Servo.class, "rightServo2");
        sensorServo = hwMap.get(Servo.class, "sensorServo");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        forkLift.setPower(0);

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        forkLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftServo.setPosition(MID_SERVO);
        //leftServo2.setPosition(MID_SERVO);
        //rightServo.setPosition(MID_SERVO);
        //rightServo2.setPosition(MID_SERVO);
        //sensorServo.setPosition(100);

    }
}



