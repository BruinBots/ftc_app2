package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.BruinHardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by rohan on 11/5/2017.
 */

@TeleOp(name="Main1", group = "Rohan")

public class Main1 extends LinearOpMode {

    static double MID_SERVO= 0.5;

    double left;
    double right;
    double drive;
    double turn;
    double max;
    double liftUp;
    double liftDown;
    public int orderedPosition = 0;

    // Steps for actions:
    // 1. Reads sensor
    // 2. Changes variable using sensor
    // 3. Fixes Range
    // 4. Moves the robot

    @Override
    public void runOpMode() {
        BruinHardware robot= new BruinHardware();


        // Initialize the servos to the mid position

        robot.init(hardwareMap);
        robot.forkLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.leftServo.setPosition(MID_SERVO);
        robot.leftServo2.setPosition(MID_SERVO);
        robot.rightServo.setPosition(MID_SERVO);
        robot.rightServo2.setPosition(MID_SERVO);
        robot.sensorServo.setPosition(0.9);
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
// ************* drive robot using gamepad *********************** //
            drive = gamepad1.left_stick_x;
            turn = 0.7*-gamepad1.right_stick_y;

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
                robot.leftWheel.setPower(-1.0);
                robot.rightWheel.setPower(-1.0);
            } else if (gamepad1.b) {
                robot.leftWheel.setPower(1.0);
                robot.rightWheel.setPower(1.0);
            } else if (gamepad1.y) {
                robot.leftWheel.setPower(1.0);
                robot.rightWheel.setPower(-1.0);
            } else if (gamepad1.x) {
                robot.leftWheel.setPower(-1.0);
                robot.rightWheel.setPower(1.0);
            }

            // Output the safe values to the motor drives.
            else {
                robot.leftWheel.setPower(left);
                robot.rightWheel.setPower(right);
            }

// ************* drive forkLift using trigger *********************** //

            liftUp = gamepad1.right_trigger;
            liftDown = -gamepad1.left_trigger;

            max = Math.max(Math.abs(liftUp), Math.abs(liftDown));
            if (max > 0.35) {
                liftUp /= max;
                liftDown /= max;
            }

          robot.forkLift.setPower(liftUp);
          robot.forkLift.setPower(liftDown);


         /*   if ((liftUp != 0) || (liftDown != 0)) { // Arm is being commanded
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
*/
// ************* open/close servoclaw with bumpers *********************** //

            //closes servo
            if (gamepad1.left_bumper) {
                robot.leftServo.setPosition(0.6);
                robot.leftServo2.setPosition(0.6);
                robot.rightServo.setPosition(0.4);
                robot.rightServo2.setPosition(0.4);//Was 0.4???
            }
            //opens servo
            else if (gamepad1.right_bumper) {
                robot.leftServo.setPosition(0.4);
                robot.leftServo2.setPosition(0.4);
                robot.rightServo.setPosition(0.6);
                robot.rightServo2.setPosition(0.6);//was 0.6???
            }

// ************* refresh data *********************** //

            telemetry.addData("Servo Position", robot.rightServo.getPosition());
            telemetry.addData("Servo Position", robot.rightServo2.getPosition());
            telemetry.addData("Servo Position", robot.leftServo.getPosition());
            telemetry.addData("Servo Position", robot.leftServo2.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();


        }


    }
}