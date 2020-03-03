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

package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.HardwareConfig_roti_negre;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Rupem Tot", group="Pushbot")

public class Mecanum_Drive_roti_negre extends OpMode{

    /* Declare OpMode members. */
     private HardwareConfig_roti_negre robot = new HardwareConfig_roti_negre();
     double coeff = .8;
    private boolean highSpeed= true;
    private double LB,LF,RF,RB;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double drive;
        double turn;
        double strafe;
        double ruleta;

        drive  = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn   = gamepad1.right_stick_x;
        ruleta = -gamepad2.right_stick_y;


        LB = -drive + strafe - turn;
        LF = -drive - strafe - turn;
        RB = -drive - strafe + turn;
        RF = -drive + strafe + turn;

        /*
        LB*=coeff;
        LF*=coeff;
        RB*=coeff;
        RF*=coeff;
        */

        if(gamepad1.x){
            highSpeed = false;
        }else if(gamepad1.b){
            highSpeed = true;
        }

        if( !highSpeed ){
            lower_power( 0.45 );
        }

        robot.leftBack.setPower(LB*coeff);
        robot.leftFront.setPower(LF*coeff);
        robot.rightFront.setPower(RF*coeff);
        robot.rightBack.setPower(RB*coeff);

        // suptere
        if(gamepad2.left_trigger>0.1){
            robot.suptStanga.setPower(1);
            robot.suptDreapta.setPower(-1);
        }else if(gamepad2.right_trigger>0.1){
            robot.suptStanga.setPower(-1);
            robot.suptDreapta.setPower(1);
        }else{
            robot.suptStanga.setPower(0);
            robot.suptDreapta.setPower(0);
        }
        //intors placa supt
        if(gamepad2.x){
            robot.intorsPlaca.setPower(0.5);
        }else if(gamepad2.b){
            robot.intorsPlaca.setPower(-0.75);
        }else{
            robot.intorsPlaca.setPower(0);
        }

        //ridicat glisiere
        if(gamepad2.dpad_up){
            robot.glisiere.setPower(-1);
        }else if(gamepad2.dpad_down){
            robot.glisiere.setPower(0.7);
        }else{
            robot.glisiere.setPower(0);
        }

        if(gamepad2.left_bumper){
            robot.intoarcere.setPosition(1);
        }else if(gamepad2.right_bumper){
            robot.intoarcere.setPosition(0.21);
        }

        if(gamepad2.dpad_left){
            robot.prins.setPosition(0.43);
        }else if(gamepad2.dpad_right){
            robot.prins.setPosition(0.65);
        }

        if(gamepad1.dpad_left){
            robot.prins_fundatie_stg.setPosition(0);//corect
            robot.prins_fundatie_dr.setPosition(0.6);
        }else if(gamepad1.dpad_right){
            robot.prins_fundatie_stg.setPosition(0.8);//corect
            robot.prins_fundatie_dr.setPosition(0);
        }

     /*   if(gamepad1.a){
            robot.claw.setPosition(0.6);
        }else if(gamepad1.y){
            robot.claw.setPosition(0.1);
        }

        if(gamepad1.right_bumper){
            robot.brat.setPosition(0);
        }else if(gamepad1.left_bumper){
            robot.brat.setPosition(1);
        }
       */
        robot.parcare.setPower(ruleta);
        if(gamepad2.a){
            robot.capstone.setPosition(0);
        }else if(gamepad2.y){
            robot.capstone.setPosition(1);
        }
        //pentru 2 miscari deodata
        /*
        if(gamepad2.dpad_right){
            robot.prins.setPosition(0.8);
            sleep(50);
            robot.intoarcere.setPosition(0.7);
            sleep(50);
        }else if(gamepad2.dpad_left){
            robot.prins.setPosition(0);
            sleep(50);
            robot.intoarcere.setPosition(0);
        }  */


        // Use gamepad buttons to move the arm up (Y) and down (A)

        // Send telemetry message to signify robot running;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void lower_power(double gain){
        LB *= gain;
        LF *= gain;
        RB *= gain;
        RF *= gain;
    }
    private void sleep(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){e.printStackTrace();}
    }

}
