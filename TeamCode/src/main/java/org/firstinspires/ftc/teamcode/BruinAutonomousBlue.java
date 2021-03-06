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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.BruinHardware;

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "RohanBruinBotAutoBlue", group = "Rohan")
//@Disabled                            // Comment this out to add to the opmode list
public class BruinAutonomousBlue extends LinearOpMode {

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
    /* Declare OpMode members. */
    BruinHardware    robot = new BruinHardware();
    ModernRoboticsI2cGyro gyro=null;
    /*ColorSensor colorSensor;
    DcMotor rightWheel;
    DcMotor leftWheel;
    Servo sensorServo;
    Servo leftServo;
    Servo rightServo;
    Servo leftServo2;
    Servo rightServo2;

    leftWheel.setDirection(DcMotor.Direction.FORWARD);
    rightWheel.setDirection(DcMotor.Direction.REVERSE);*/

    //Set-up omni wheels 7in away; normal wheels 4 1/2in away

    //DistanceSensor sensorDistance;
    private ElapsedTime runtime = new ElapsedTime();

    public void turnLeft (double turnSpeed, double seconds) {
        robot.leftWheel.setPower(-turnSpeed);
        robot.rightWheel.setPower(turnSpeed);
        sleep(Math.round(seconds*1000));
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        //runtime.reset();
        //while (opModeIsActive() && (runtime.seconds() < seconds)) {
          //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        //}
    }

    public void turnRight (double turnSpeed, double seconds) {
        robot.leftWheel.setPower(-turnSpeed);
        robot.rightWheel.setPower(turnSpeed);
        sleep(Math.round(seconds*1000));
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        //runtime.reset();
        //while (opModeIsActive() && (runtime.seconds() < seconds)) {
          //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        //}
    }

    public void goForward (double frwrdSpeed, double seconds) {
        robot.leftWheel.setPower(frwrdSpeed);
        robot.rightWheel.setPower(frwrdSpeed);
        sleep(Math.round(seconds*1000));
        robot.leftWheel.setPower(0);
        robot.rightWheel.setPower(0);
        //This is untested... test it tomorrow @ tournament.
    }


    @Override
    public void runOpMode() {

        // get a reference to the color sensor.
        /*colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        sensorServo = hardwareMap.get(Servo.class, "sensorServo");
        leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo2 = hwMap.get(Servo.class, "leftServo2");
        rightServo2 = hwMap.get(Servo.class, "rightServo2");*/
        robot.init(hardwareMap);


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

        //robot.sensorServo.setPosition(0);

        robot.leftServo.setPosition(0.4);
        robot.leftServo2.setPosition(0.4);
        robot.rightServo.setPosition(0.6);
        robot.rightServo2.setPosition(0.6);


        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        //while (opModeIsActive()) {
        robot.sensorServo.setPosition(0.15);

            sleep(3000);

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                    (int) (robot.colorSensor.green() * SCALE_FACTOR),
                    (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            if (robot.colorSensor.red() < robot.colorSensor.blue()) {
                telemetry.addData("Blue!!!!  ", robot.colorSensor.red());
                turnLeft (0.15, 0.5);
                sleep(1000);
                robot.sensorServo.setPosition(0.9);
                sleep(1000);
                turnRight(0.15, 0.5);
                sleep(1000);
                robot.leftWheel.setPower(0);
                robot.rightWheel.setPower(0);
            }
            else {
                telemetry.addData("Red!!!!  ", robot.colorSensor.blue());
                turnRight(0.15, 0.5);
                sleep(1000);
                robot.sensorServo.setPosition(0.9);
                sleep(1000);
                turnLeft(0.15, 0.5);
                sleep(1000);
                robot.leftWheel.setPower(0);
                robot.rightWheel.setPower(0);
            }
            sleep(2000);
            turnLeft (0.5, 0.6);
            sleep(1000);
            goForward(1,0.5);


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

