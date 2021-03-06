package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareBruinbot
{
    /* Public OpMode members. */
    public DcMotor leftWheel   = null;
    public DcMotor rightWheel  = null;
    public DcMotor forkLift    = null;
    public Servo leftServo    = null;
    public Servo rightServo   = null;
    public Servo sensorServo = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBruinbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        // Save reference to Hardware map
        hwMap = hwMap;

        // Define and Initialize Motors
        leftWheel   = hwMap.dcMotor.get("leftWheel");
        rightWheel  = hwMap.dcMotor.get("rightWheel");
        forkLift    = hwMap.dcMotor.get("forkLift");
        leftWheel.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightWheel.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        forkLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        forkLift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        forkLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // forkLift.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);

        // Define and initialize ALL installed servos.
        leftServo = hwMap.servo.get("leftServo");
        rightServo = hwMap.servo.get("rightServo");
        leftServo.setPosition(MID_SERVO);
        rightServo.setPosition(MID_SERVO);

        sensorServo = hwMap.servo.get("sensorServo");
        sensorServo.setPosition(0);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

