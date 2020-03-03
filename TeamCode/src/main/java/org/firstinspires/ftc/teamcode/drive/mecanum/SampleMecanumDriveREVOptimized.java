package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import android.graphics.Color;
import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx hub;
    private ExpansionHubMotor leftFront, leftBack, rightBack, rightFront;
    public DcMotor   suptStanga  = null;
    public DcMotor   suptDreapta = null;
    public DcMotor   intorsPlaca = null;
    public DcMotor   glisiere    = null;
    private BNO055IMU imu         = null;
    private Servo     prins       = null;
    private Servo     intoarcere  = null;
    public Servo     prins_fundatie_dr   = null;
    public Servo     prins_fundatie_stg    = null;
    public Servo     brat    = null;
    public Servo     claw    = null;
    protected   CRServo  parcare = null;
    public ColorSensor  led      = null;

    private List<ExpansionHubMotor> motors;

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**

        hub = hardwareMap.get(ExpansionHubEx.class, "control_hub");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
         BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "left_front");
        leftBack = hardwareMap.get(ExpansionHubMotor.class, "left_back");
        rightBack = hardwareMap.get(ExpansionHubMotor.class, "right_back");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "right_front");
        suptDreapta = hardwareMap.get(DcMotor.class, "supt_dreapta");
        suptStanga  = hardwareMap.get(DcMotor.class, "supt_stanga");
        intorsPlaca = hardwareMap.get(DcMotor.class, "intors_placa");
        glisiere    = hardwareMap.get(DcMotor.class, "glisiere");
        prins       = hardwareMap.get(Servo.class, "prins");
        intoarcere  = hardwareMap.get(Servo.class, "intoarcere");
        prins_fundatie_dr   = hardwareMap.get(Servo.class, "prins_fundatie_dr");
        prins_fundatie_stg    = hardwareMap.get(Servo.class, "prins_fundatie_stg");
        parcare               = hardwareMap.get(CRServo.class, "parcare");
        brat        = hardwareMap.get(Servo.class,"brat");
        claw        = hardwareMap.get(Servo.class,"claw");
        led          = hardwareMap.get(ColorSensor.class, "led");
        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (ExpansionHubMotor motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
              //leftBack.setDirection(DcMotorEx.Direction.REVERSE);
              // leftFront.setDirection(DcMotorEx.Direction.REVERSE);
               rightFront.setDirection(DcMotorEx.Direction.REVERSE);
               rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
         setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(27.5,0,14));
         led.enableLed(false);
        // TODO: if desired, use setLocalizer() to change the localization method
        //for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
