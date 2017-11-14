package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by rohan on 11/5/2017.
 */

@TeleOp(name="Main1", group = "Rohan")

public class Main1 extends LinearOpMode {
    private Servo leftServo;
    private Servo rightServo;
    private DcMotor forkLift;
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    double left;
    double right;
    double drive;
    double turn;
    double max;
    double liftUp;
    double liftDown;
    public int orderedPosition = 0;
    public static final double MID_SERVO = 0.5;
    // Steps for actions:
    // 1. Reads sensor
    // 2. Changes variable using sensor
    // 3. Fixes Range
    // 4. Moves the robot

    @Override
    public void runOpMode() {
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
        forkLift = hardwareMap.get(DcMotor.class, "forkLift");
        //This section resets the forkLift... The forkLift MUST start in lowest position!
        forkLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Resets encoders
        while (forkLift.getCurrentPosition() != 0) { //Ensures encoders are zero
            forkLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.update(); //Needed within all loops
        }
        forkLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Sets mode to use encoders setMode() is used instead of setChannelMode(), which is now deprecated
        forkLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //forkLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize the servos to the mid position
        leftServo.setPosition(MID_SERVO);
        rightServo.setPosition(MID_SERVO);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
// ************* drive robot using gamepad *********************** //
            drive = gamepad1.right_stick_x;
            turn = -gamepad1.left_stick_y;

            // Combine drive and turn for blended motion.
            left = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            // Nitro boost to wheels: full speed
            if (gamepad1.a) {
                leftWheel.setPower(1.0);
                rightWheel.setPower(-1.0);
            } else if (gamepad1.b) {
                leftWheel.setPower(-1.0);
                rightWheel.setPower(1.0);
            } else if (gamepad1.y) {
                leftWheel.setPower(1.0);
                rightWheel.setPower(1.0);
            } else if (gamepad1.x) {
                leftWheel.setPower(-1.0);
                rightWheel.setPower(-1.0);
            }

            // Output the safe values to the motor drives.
            else {
                leftWheel.setPower(left);
                rightWheel.setPower(right);
            }

// ************* drive forkLift using trigger *********************** //

            liftUp = gamepad1.right_trigger;
            liftDown = -gamepad1.left_trigger;

            max = Math.max(Math.abs(liftUp), Math.abs(liftDown));
            if (max > 0.35) {
                liftUp /= max;
                liftDown /= max;
            }
        /*
          forkLift.setPower(liftUp);
          forkLift.setPower(liftDown);
*/
            if ((liftUp != 0) || (liftDown != 0)) { // Arm is being commanded
                if (liftUp > liftDown) {  // Arm is commanded upwards
                    orderedPosition += 500;
                    forkLift.setTargetPosition(orderedPosition); //Sets motor to move 1440 ticks (1440 is one rotation for Tetrix motors)
                    forkLift.setPower(0.5);
                    while (forkLift.getCurrentPosition() < forkLift.getTargetPosition()) { //While target has not been reached
                        telemetry.addData("ForkLift Position", forkLift.getCurrentPosition());
                        telemetry.update(); //Needed within all loops
                    }
                    forkLift.setPower(0);
                } else {

                    if (orderedPosition>0) {
                        orderedPosition -= 500;
                    }
                    forkLift.setTargetPosition(orderedPosition); //Sets motor to move 1440 ticks (1440 is one rotation for Tetrix motors)
                    forkLift.setPower(-0.5);
                    while (forkLift.getCurrentPosition() > forkLift.getTargetPosition()) { //While target has not been reached
                        telemetry.addData("ForkLift Position", forkLift.getCurrentPosition());
                        telemetry.update(); //Needed within all loops
                    }
                    forkLift.setPower(0);
                }
            }
// ************* open/close servoclaw with bumpers *********************** //

            //closes servo
            if (gamepad1.left_bumper) {
                leftServo.setPosition(0.6);
                rightServo.setPosition(0.4);
            }
            //opens servo
            else if (gamepad1.right_bumper) {
                leftServo.setPosition(0.4);
                rightServo.setPosition(0.6);
            }

// ************* refresh data *********************** //

            telemetry.addData("Servo Position", rightServo.getPosition());
            telemetry.addData("Servo Position", leftServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();


        }


    }
}