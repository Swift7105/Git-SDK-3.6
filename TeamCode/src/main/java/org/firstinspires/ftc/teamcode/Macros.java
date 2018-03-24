package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.*;

import static android.os.SystemClock.sleep;


/**
 * Created by FRC on 3/22/2018.
 */
@Autonomous(name = "Macros", group = "Macros")
public class Macros extends Command {
    CompetitionHWsetup robot = new CompetitionHWsetup();// use the class created to define a Pushbot's hardware

    public Macros() {
        super("Macros");
    }


    public void CubeArmOut(){
        LiftEncoder(0.5,10);
        ArmEncoder(0.5,2);
        
        

    }
  
    public void ArmStop (){
        robot.arm.setPower(0);
    }








    //---------------------------------------------------------------------------------\\


    //Allows the ability to run the Mechanum as a tank drive using the encoders to run to a spcific distance at a cetain speed.
    public void ArmEncoder (double armpower, int armdistance){


        //sets the encoder values to zero
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        //sets the position(distance) to drive to
        robot.arm.setTargetPosition(armdistance);
       

        //engages the encoders to start tracking revolutions of the motor axel
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        

        //powers up the left and right side of the drivetrain independently
        robot.arm.setPower(armpower);

        //will pause the program until the motors have run to the previously specified position
        while (robot.arm.isBusy())
        {

        }

        //stops the motors and sets them back to normal operation mode
        LiftStop();
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
    }



//----------------------------------------------------------------------------------------\\




    //Stops all motors in the drivetrain
    public void LiftStop (){
        robot.arm.setPower(0);
    }


    //Allows the ability to run the Mechanum as a tank drive using the encoders to run to a spcific distance at a cetain speed.
    public void LiftEncoder (double armpower, int armdistance){


        //sets the encoder values to zero
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //sets the position(distance) to drive to
        robot.arm.setTargetPosition(armdistance);


        //engages the encoders to start tracking revolutions of the motor axel
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //powers up the left and right side of the drivetrain independently
        robot.arm.setPower(armpower);

        //will pause the program until the motors have run to the previously specified position
        while (robot.arm.isBusy())
        {

        }

        //stops the motors and sets them back to normal operation mode
        LiftStop();
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    
}
