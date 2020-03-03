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

package org.firstinspires.ftc.teamcode.Auto_Routines;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.HardwareConfig_roti_negre;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="placa rosu tras encodere", group="Pushbot")

public class Auto_rosu_placa_enc extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareConfig_roti_negre robot = new HardwareConfig_roti_negre();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
    //private static final double TICKS_PER_REV = 1120;
    private static final double     DRIVE_GEAR_REDUCTION    = 0.6666 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    private static double DRIVE_SPEED = 0.5;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        robot.imu.initialize(parameters);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        resetEncoders();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftBack.getCurrentPosition(),
                          robot.leftFront.getCurrentPosition(),
                          robot.rightFront.getCurrentPosition(),
                          robot.rightBack.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // ro128: fata ---- spate ++++ strafe stanga -++- strafe dreapta +--+
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(0.9, -14,14,14,-14,4);
        sleep(100);
        encoderDrive(0.6, 30,30,30,30,4);
        encoderDrive(0.4, 7,7,7,7,4);
        robot.prins_fundatie_stg.setPosition(0);
        robot.prins_fundatie_dr.setPosition(0.6);
        sleep(300);
        encoderDrive(0.5, -30,-30,-30,-30,4);
        //intors odata cu placa
        encoderDrive(0.4, -35,-35,35,35,4);
        robot.prins_fundatie_stg.setPosition(0.8);
        robot.prins_fundatie_dr.setPosition(0);
        sleep(300);
        encoderDrive(0.8,2,2,2,2,3);
        sleep(200);
        encoderDrive(0.9,-12,12,12,-12,3);
        sleep(300);
        encoderDrive(0.9,-47,-47,-47,-47,4);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftBackInches , double leftFrontInches,
                                           double rightBackInches, double rightFrontInches,
                                           double timeoutS) {
        int LBT,LFT,RBT,RFT;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LBT = robot.leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            LFT = robot.leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            RBT = robot.rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            RFT = robot.rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);

            robot.leftBack.setTargetPosition(LBT);
            robot.leftFront.setTargetPosition(LFT);
            robot.rightBack.setTargetPosition(RBT);
            robot.rightFront.setTargetPosition(RFT);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftBack.setPower(Math.abs(speed));
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFront.isBusy() && robot.rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", LFT,  RFT);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            robot.leftBack.getCurrentPosition(),
                                            robot.leftFront.getCurrentPosition(),
                                            robot.rightFront.getCurrentPosition(),
                                            robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stopPower();
            // Turn off RUN_TO_POSITION;
            sleep(100);   // optional pause after each move\\
            resetEncoders();
        }

    }
    public void resetEncoders(){
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopPower(){
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void gyro_turn(double angle, double power){
        Orientation orientare = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double initial = orientare.firstAngle;
        if(angle > 0){
            while(orientare.firstAngle - initial < angle*2/3){
                robot.leftFront.setPower(power);
                robot.leftBack.setPower(power);
                robot.rightBack.setPower(-power);
                robot.rightFront.setPower(-power);
            }
            while(orientare.firstAngle - initial> angle*2/3 && orientare.firstAngle - initial< angle){
                robot.leftFront.setPower(power/2);
                robot.leftBack.setPower(power/2);
                robot.rightBack.setPower(-power/2);
                robot.rightFront.setPower(-power/2);
            }
        }
        else if(angle < 0){
            while(orientare.firstAngle - initial > angle*2/3){
                robot.leftFront.setPower(-power);
                robot.leftBack.setPower(-power);
                robot.rightBack.setPower(power);
                robot.rightFront.setPower(power);
            }
            while(orientare.firstAngle -initial < angle*2/3 && orientare.firstAngle - initial > angle){
                robot.leftFront.setPower(-power/2);
                robot.leftBack.setPower(-power/2);
                robot.rightBack.setPower(power/2);
                robot.rightFront.setPower(power/2);
            }
            stopPower();
        }
        else return;
    }
}
