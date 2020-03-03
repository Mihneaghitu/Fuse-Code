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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

import java.security.CryptoPrimitive;
import java.util.Arrays;
import java.util.List;

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
public class HardwareConfig_roti_negre
{
    /* Public OpMode members. */
    public DcMotor   leftBack           = null;
    public DcMotor   leftFront          = null;
    public DcMotor   rightFront         = null;
    public DcMotor   rightBack          = null;
    public DcMotor   suptStanga         = null;
    public DcMotor   suptDreapta        = null;
    public DcMotor   intorsPlaca        = null;
    public DcMotor   glisiere           = null;
    public BNO055IMU imu                = null;
    public Servo     prins              = null;
    public Servo     intoarcere         = null;
    public Servo     prins_fundatie_dr  = null;
    public Servo     prins_fundatie_stg = null;
    public Servo     brat               = null;
    public Servo     claw               = null;
    public CRServo   parcare            = null;
    public ColorSensor led              = null;
    public Servo     capstone           = null;

    public List<DcMotor> motoare;
    public List<Servo> servouri;
    public List<CRServo> crservouri;
    public Servo  ghiara =null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  =  new ElapsedTime();

    /* Constructor */
    public HardwareConfig_roti_negre(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftBack           = hwMap.get(DcMotor.class, "left_back");
        leftFront          = hwMap.get(DcMotor.class, "left_front");
        rightFront         = hwMap.get(DcMotor.class, "right_front");
        rightBack          = hwMap.get(DcMotor.class, "right_back");
        suptDreapta        = hwMap.get(DcMotor.class, "supt_dreapta");
        suptStanga         = hwMap.get(DcMotor.class, "supt_stanga");
        intorsPlaca        = hwMap.get(DcMotor.class, "intors_placa");
        glisiere           = hwMap.get(DcMotor.class, "glisiere");
        imu                = hwMap.get(BNO055IMU.class, "imu");
        prins              = hwMap.get(Servo.class, "prins");
        //prinsstg    = hwMap.get(CRServo.class, "prinsdr");
        intoarcere         = hwMap.get(Servo.class, "intoarcere");
        prins_fundatie_dr  = hwMap.get(Servo.class,"prins_fundatie_dr");
        prins_fundatie_stg = hwMap.get(Servo.class,"prins_fundatie_stg");
        parcare            = hwMap.get(CRServo.class, "parcare");
        brat               = hwMap.get(Servo.class, "brat");
        claw               = hwMap.get(Servo.class, "claw");
        led                = hwMap.get(ColorSensor.class, "led");
        capstone           = hwMap.get(Servo.class,"capstone");

        leftBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        suptStanga.setDirection(DcMotor.Direction.FORWARD);
        suptDreapta.setDirection(DcMotor.Direction.FORWARD);
        intorsPlaca.setDirection(DcMotor.Direction.FORWARD);
        glisiere.setDirection(DcMotor.Direction.FORWARD);
        //prinsdr.setDirection(CRServo.Direction.FORWARD);
        //prinsstg.setDirection(CRServo.Direction.REVERSE);

        motoare  = Arrays.asList(leftBack,leftFront,rightBack,rightFront,suptDreapta,suptStanga,intorsPlaca,glisiere);
        servouri = Arrays.asList(intoarcere,prins,prins_fundatie_stg,prins_fundatie_dr,brat,claw);
        //crservouri = Arrays.asList(prinsdr,prinsstg);
        // Define and initialize ALL installed servos

        for (DcMotor motor: motoare){
            // Set all motors to zero power
            motor.setPower(0);
            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //Set zero power behavior
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //for (Servo servo : servouri){
          //  servo.setPosition(0.5);
        //}
        intoarcere.setPosition(0.21);
        prins_fundatie_dr.setPosition(0);
        prins_fundatie_stg.setPosition(0.8);
        claw.setPosition(0);
        brat.setPosition(.4);
       /* for (CRServo crservo : crservouri){
            crservo.setPower(0);
        } */
       parcare.setPower(0);

    }
 }

