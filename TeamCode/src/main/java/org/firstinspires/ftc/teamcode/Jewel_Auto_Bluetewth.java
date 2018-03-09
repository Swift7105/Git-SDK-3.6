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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Jewel Auto Bluetewth", group="Pushbot")
@Disabled
public class Jewel_Auto_Bluetewth extends LinearOpMode {

    /* Declare OpMode members. */
   CompetitionHWsetup robot = new CompetitionHWsetup();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");



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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        boolean looking = true;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);







        robot.jewelarm.setPosition(0);
        sleep(2000);

        //----------------------
        if (looking = true)  {
            if (sensorColor.red() > sensorColor.blue()) {
                robot.backleftMotor.setPower(.25);
                robot.frontleftMotor.setPower(.25);
                robot.backrightMotor.setPower(-.25);
                robot.frontrightMotor.setPower(-.25);
                sleep(500);
                robot.backleftMotor.setPower(0);
                robot.frontleftMotor.setPower(0);
                robot.backrightMotor.setPower(0);
                robot.frontrightMotor.setPower(0);
                sleep(100);
                robot.jewelarm.setPosition(1);
                sleep(2000);
                robot.backleftMotor.setPower(-.25);
                robot.frontleftMotor.setPower(-.25);
                robot.backrightMotor.setPower(.25);
                robot.frontrightMotor.setPower(.25);
                sleep(500);
                robot.backleftMotor.setPower(0);
                robot.frontleftMotor.setPower(0);
                robot.backrightMotor.setPower(0);
                robot.frontrightMotor.setPower(0);
                sleep(100);
                robot.backleftMotor.setPower(-.25);
                robot.frontleftMotor.setPower(-.25);
                robot.backrightMotor.setPower(-.25);
                robot.frontrightMotor.setPower(-.25);
                sleep(500);
                robot.backleftMotor.setPower(0);
                robot.frontleftMotor.setPower(0);
                robot.backrightMotor.setPower(0);
                robot.frontrightMotor.setPower(0);
                sleep(100);
                looking = false;


            } else if (sensorColor.blue() > sensorColor.red()) {
                robot.backleftMotor.setPower(-.25);
                robot.frontleftMotor.setPower(-.25);
                robot.backrightMotor.setPower(-.25);
                robot.frontrightMotor.setPower(-.25);
                sleep(500);
                robot.backleftMotor.setPower(0);
                robot.frontleftMotor.setPower(0);
                robot.backrightMotor.setPower(0);
                robot.frontrightMotor.setPower(0);
                sleep(100);
                robot.jewelarm.setPosition(1);
                sleep(2000);


                looking = false;
            }

            //--------------------------------

            sleep(100);
            robot.jewelarm.setPosition(0.6);
            sleep(1000);
            robot.backleftMotor.setPower(-.25);
            robot.frontleftMotor.setPower(-.25);
            robot.backrightMotor.setPower(-.25);
            robot.frontrightMotor.setPower(-.25);
            sleep(900);
            robot.backleftMotor.setPower(0);
            robot.frontleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);
            robot.frontrightMotor.setPower(0);
            sleep(100);

            robot.backleftMotor.setPower(-.25);
            robot.frontleftMotor.setPower(-.25);
            robot.backrightMotor.setPower(-.25);
            robot.frontrightMotor.setPower(-.25);
            sleep(1000);
            robot.backleftMotor.setPower(0);
            robot.frontleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);
            robot.frontrightMotor.setPower(0);
            sleep(100);
            robot.intakeright.setPower(-50);
            robot.backleftMotor.setPower(.25);
            robot.frontleftMotor.setPower(.25);
            robot.backrightMotor.setPower(-.25);
            robot.frontrightMotor.setPower(-.25);
            sleep(925);
            robot.intakeright.setPower(0);
            robot.backleftMotor.setPower(0);
            robot.frontleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);
            robot.frontrightMotor.setPower(0);
            sleep(100);
            robot.backleftMotor.setPower(.25);
            robot.frontleftMotor.setPower(.25);
            robot.backrightMotor.setPower(-.25);
            robot.frontrightMotor.setPower(-.25);
            sleep(1000);
            robot.backleftMotor.setPower(0);
            robot.frontleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);
            robot.frontrightMotor.setPower(0);
            sleep(100);
            robot.backleftMotor.setPower(-.25);
            robot.frontleftMotor.setPower(-.25);
            robot.backrightMotor.setPower(.25);
            robot.frontrightMotor.setPower(.25);
            sleep(1000);
            robot.backleftMotor.setPower(0);
            robot.frontleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);
            robot.frontrightMotor.setPower(0);
            sleep(100);
            robot.backleftMotor.setPower(.25);
            robot.frontleftMotor.setPower(.25);
            robot.backrightMotor.setPower(-.25);
            robot.frontrightMotor.setPower(-.25);
            sleep(500);
            robot.backleftMotor.setPower(0);
            robot.frontleftMotor.setPower(0);
            robot.backrightMotor.setPower(0);
            robot.frontrightMotor.setPower(0);
            sleep(100);

        }





        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


        telemetry.update();
        sleep(1000);
    }
}
