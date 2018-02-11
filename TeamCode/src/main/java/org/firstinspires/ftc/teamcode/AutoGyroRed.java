/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "AutoGyroRed", group = "Rohan")
//@Disabled                            // Comment this out to add to the opmode list
public class AutoGyroRed extends LinearOpMode {

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */

    ModernRoboticsI2cGyro sensorGyro;


    /* Declare OpMode members. */
    BruinHardware    robot = new BruinHardware();
    ModernRoboticsI2cGyro gyro=null;

    private ElapsedTime period  = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    public void goForward (double frwrdSpeed, double rotations) {
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;

        if (opModeIsActive())
        robot.leftWheel.setPower(frwrdSpeed);
        robot.rightWheel.setPower(-frwrdSpeed);
        sleep(Math.round(rotations*1000));
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    //DistanceSensor sensorDistance;
    private ElapsedTime runtime = new ElapsedTime();

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftWheel.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightWheel.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftWheel.setTargetPosition(newLeftTarget);
            robot.rightWheel.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftWheel.setPower(Math.abs(speed));
            robot.rightWheel.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftWheel.isBusy() && robot.rightWheel.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftWheel.getCurrentPosition(),
                        robot.rightWheel.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftWheel.setPower(0);
            robot.rightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     *  speed     Desired speed of turn.
     *  angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     *  holdTime   Length of time (in seconds) to hold the specified heading.
     */

    /*public void turnLeft (double turnSpeed, double angle) {

        leftWheel.setPower(-turnSpeed);
        rightWheel.setPower(-turnSpeed);

        leftWheel.setPower(0);
        rightWheel.setPower(0);
        //runtime.reset();
        //while (opModeIsActive() && (runtime.seconds() < seconds)) {
          //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        //}
    }

    public void turnRight (double turnSpeed, double seconds) {
        leftWheel.setPower(turnSpeed);
        rightWheel.setPower(turnSpeed);
        sleep(Math.round(seconds*1000));
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        //runtime.reset();
        //while (opModeIsActive() && (runtime.seconds() < seconds)) {
          //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        //}
    }
*/

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftWheel.setPower(leftSpeed);
        robot.rightWheel.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    @Override
    public void runOpMode() {

        // get a reference to the color sensor.
        sensorGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "sensorGyro");

        telemetry.addData(">", "Robot Set.");
        telemetry.update();

        sensorGyro.calibrate();

        while (!isStopRequested() && sensorGyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", sensorGyro.getIntegratedZValue());
            telemetry.update();
        }

        sensorGyro.resetZAxisIntegrator();

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // wait for the start button to be pressed.
        waitForStart();

        robot.sensorServo.setPosition(0);

        robot.leftServo.setPosition(0.4);
        robot.rightServo.setPosition(0.6);
        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        //while (opModeIsActive()) {
            robot.sensorServo.setPosition(0.67);
            sleep(3000);

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                    (int) (robot.colorSensor.green() * SCALE_FACTOR),
                    (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            if (robot.colorSensor.red() > robot.colorSensor.blue()) {
                telemetry.addData("Red!!!!  ", robot.colorSensor.red());
                gyroTurn (0.15, -10);
                sleep(1000);
                robot.sensorServo.setPosition(0);
                sleep(1000);
                gyroTurn(0.15, 10);
                sleep(1000);
                robot.leftWheel.setPower(0);
                robot.rightWheel.setPower(0);
            }
            else {
                telemetry.addData("Blue!!!!  ", robot.colorSensor.blue());
                gyroTurn(0.15, 10);
                sleep(1000);
                robot.sensorServo.setPosition(0);
                sleep(1000);
                gyroTurn(0.15, -10);
                sleep(1000);
                robot.leftWheel.setPower(0);
                robot.rightWheel.setPower(0);
            }
            sleep(2000);
            gyroTurn(0.5, 90);
            sleep(1000);
            goForward(1, 0.5);

            telemetry.update();
            sleep(1000);
            robot.leftWheel.setPower(0);
            robot.rightWheel.setPower(0);



            // send the info back to driver station using telemetry function.
            //telemetry.addData("Distance (cm)",
                  //  String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", robot.colorSensor.alpha());
            telemetry.addData("Red  ", robot.colorSensor.red());
            telemetry.addData("Green", robot.colorSensor.green());
            telemetry.addData("Blue ", robot.colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        //}

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
        sleep(1000);
    }
}

