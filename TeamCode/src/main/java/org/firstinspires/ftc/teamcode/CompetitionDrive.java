/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Pushbot: CompetitionDrive", group="Pushbot")
//@Disabled
public class CompetitionDrive extends OpMode{
    int relicdistace;
      boolean relicclaw;
    /* Declare OpMode members. */
    CompetitionHWsetup robot = new CompetitionHWsetup();// use the class created to define a Pushbot's hardware


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        relicclaw = false;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        
        robot.jewelarm.setPosition(0.9);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { }
    /*
     * Code to run ONCE when the driver hits PLAY
     *
     */


    @Override
    public void start() { }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;

        final double v1 = r * Math.sin(robotAngle) - rightX;
        final double v2 = r * Math.cos(robotAngle) + rightX;
        final double v3 = r * Math.cos(robotAngle) - rightX;
        final double v4 = r * Math.sin(robotAngle) + rightX;

        if (gamepad1.left_bumper){
            robot.frontleftMotor.setPower(v1 / 2);
            robot.frontrightMotor.setPower(v2 / 2);
            robot.backleftMotor.setPower(v3 / 2);
            robot.backrightMotor.setPower(v4 / 2);
        }
        else {
            robot.frontleftMotor.setPower(v1 * v1 * v1 * 5);
            robot.frontrightMotor.setPower(v2 * v2 * v2 * 5 );
            robot.backleftMotor.setPower(v3 * v3 * v3 * 5);
            robot.backrightMotor.setPower(v4 * v4 * v4 * 5);
        }

/*
        robot.frontleftMotor.setPower(v1 * v1 * v1 * 1.50);
        robot.frontrightMotor.setPower(v2 * v2 * v2 * 1.50 );
        robot.backleftMotor.setPower(v3 * v3 * v3 * 1.50);
        robot.backrightMotor.setPower(v4 * v4 * v4 * 1.50);

        if ((gamepad1.left_bumper) ){
            robot.frontleftMotor.setPower(v1 * 0.25);
            robot.frontrightMotor.setPower(v2 * 0.25);
            robot.backleftMotor.setPower(v3 * 0.25);
            robot.backrightMotor.setPower(v4 * 0.25);
        }
*/


        //intake

        if ((gamepad2.right_trigger) > 0.1){     //intake
            robot.intakeright.setPower(- gamepad2.right_trigger);
        }
        else if ((gamepad2.right_bumper) ){      // outtake
            robot.intakeright.setPower(1);
        }
        else if (((gamepad2.right_trigger) < 0.1) || ((gamepad2.right_bumper) = false)) {
            robot.intakeright.setPower(0);
        }

          /*
        if ((gamepad2.left_trigger) > .1){      // intake
            robot.intakeleft.setPower(gamepad2.left_trigger * -.75 );
        }
        else if ((gamepad2.left_bumper) ){      // outtake
            robot.intakeleft.setPower( 50);
        }
        else if (((gamepad2.left_trigger) < 0.1) || ((gamepad2.left_bumper) = false)){
            robot.intakeleft.setPower(0);
        }
         */

        //claw servos


        if (gamepad2.y){
            //open
            robot.frontclaw.setPosition(0.01);
           robot.backclaw.setPosition(0.99);
            robot.bigclaw.setPosition(0.99);

        }
        else if (gamepad2.x) {
            //close
            robot.frontclaw.setPosition(0.99);
            robot.backclaw.setPosition(0.01);
            robot.bigclaw.setPosition(0.01);
        }

        if (gamepad1.right_bumper){
            robot.frontclaw.setPosition(0.01);
            robot.backclaw.setPosition(0.99);
            robot.bigclaw.setPosition(0.99);
        }


        //claw lift and arm

        double lift;
        double arm;

        lift = (gamepad2.left_stick_y);
        arm = (gamepad2.right_stick_y);

        robot.lift.setPower(lift);
        robot.arm.setPower(arm * -0.85);

        telemetry.addData("X" ,(arm));
        telemetry.addData("Y" , lift);
        telemetry.addData("BumperRight", (gamepad2.right_bumper));




      //  robot.relicarm.setPower(-gamepad2.right_trigger * 5);
        //robot.relicarm.setPower(gamepad2.left_trigger * 5);

        if ( gamepad2.dpad_up){
            robot.relicarm.setPower(1);

        }

        else if ( gamepad2.dpad_down){
            robot.relicarm.setPower(-1);

        }
        else {
            robot.relicarm.setPower(0);
        }
       /* if (gamepad2.dpad_up){
           relicarm(0.5, -280);
        }*/


        /*

        else if (gamepad2.dpad_right) {
            robot.relicarm.setPower(-0.25);

        }*/

        if (gamepad2.b) {
            robot.relicclaw.setPosition(0.5);
        }
        if (gamepad2.a){
                robot.relicclaw.setPosition(0.9);
        }
        if (gamepad2.left_bumper){
            robot.relicclaw.setPosition(0.8);
        }
        if (gamepad2.dpad_right){
            robot.relicclaw.setPosition(1);
        }

/*


        if power(postionthatyouwant - positionactual)*.7
            if power < 0
                power = power * 10
*/



   /*     //threshold is the deadzone.  The bigger the number the bigger the deadzone
        double threshold = 20; 

        if(abs (gamepad1.right_stick_y + gamepad1.right_stick_x) > threshold || abs(gamepad1.right_stick_x) > threshold){

            robot.frontrightMotor.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x));
            robot.backleftMotor.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x));

            robot.frontleftMotor.setPower((-gamepad1.right_stick_y - gamepad1.right_stick_x));
            robot.backrightMotor.setPower((-gamepad1.right_stick_y - gamepad1.right_stick_x));
        }

        if(abs(gamepad1.left_stick_x) > threshold){
            robot.frontrightMotor.setPower((-gamepad1.left_stick_x));
            robot.frontleftMotor.setPower((-gamepad1.left_stick_x));

            robot.backrightMotor.setPower((gamepad1.left_stick_x));
            robot.backleftMotor.setPower((gamepad1.left_stick_x));
        }
*/




        updateTelemetry(telemetry);
    }





    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
