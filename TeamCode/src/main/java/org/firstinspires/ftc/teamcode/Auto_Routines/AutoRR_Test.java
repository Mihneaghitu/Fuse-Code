package org.firstinspires.ftc.teamcode.Auto_Routines;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.math.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */

@Autonomous(name="RR Test", group = "drive")

public class AutoRR_Test extends LinearOpMode {
    public static double DISTANCE = 30;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private TFObjectDetector tfod;
    private static final String VUFORIA_KEY ="AdH+fOf/////AAABmdAPFiAA0UkHg0wrxd6vkF2HtfaXeSQ+TQdw15u05/xF5Ic5vxE8DcJ0azg55zUkyWKic8DCzvfTQArVre8JVcWzdkCxxBDJYJba0Ks1CDAshM6fuoxNOHbfExHpL0dQZJA7DNTEg0Fvkg3ogfnN3tNgn6JTfgjmWySShIOD6RjW+7hJ251c8LBTHJCAvTbKVGGulNBgjwO+w1bj+9/oBSA1QGISIYBWIEYpM+smYt0LamOjUVHFPmWcmIT6g/NgfhU7IfDxR2DbXHsSxvOIo0fFJCwbiHE/jgTbd8s6oHBOIgFZe9Fs/yJ/GmJZixpsl1zP049X6rnW73wzZvh6puxTXdultfbdEn8nCEzaYAiu";
    private VuforiaLocalizer vuforia;
    public ElapsedTime runtime = new ElapsedTime();
    //Trajectories traiectorii_robot = new Trajectories();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized robot = new SampleMecanumDriveREVOptimized(hardwareMap);

        initVuforia();
        robot.led.enableLed(true);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        tfod.setClippingMargins(20,180,110,0);
        Trajectory trajectory = robot.trajectoryBuilder()
                .strafeLeft(21.5)
                .build();

        waitForStart();

        if (isStopRequested()) return;
        robot.claw.setPosition(0.65); //pe spate
        //robot.brat.setPosition(0.7);  //in robot
        robot.brat.setPosition(0);
        robot.prins_fundatie_stg.setPosition(0.8);
        robot.prins_fundatie_dr.setPosition(0);
        robot.followTrajectorySync(trajectory);
        sleep(450);
        String position = detecteaza();
        runtime.reset();
        while(runtime.seconds() < 1.0){
            telemetry.addData("pozitie", position);
            telemetry.update();
        }

        position = "dreapta";
       if(position == "dreapta") traiectoriedrp(robot);
       else if(position == "stanga") traiectoriestg(robot);
       else traiectoriemij(robot);

    }

    private void traiectoriestg(SampleMecanumDriveREVOptimized robot){
        Trajectory traj0 = robot.trajectoryBuilder()
                .back(10)
                .build();
        robot.followTrajectorySync(traj0);

    }

    private void traiectoriedrp(SampleMecanumDriveREVOptimized robot){
        Trajectory traj0 = robot.trajectoryBuilder()
                .forward(1)
                .strafeLeft(14)
                .build();
        robot.followTrajectorySync(traj0);
        robot.claw.setPosition(0);
        sleep(100);
        Trajectory traj1 = robot.trajectoryBuilder()
                .reverse()
                .addMarker(()->{
                    robot.brat.setPosition(0.35);
                    return null;
                })
                .splineTo(new Pose2d(-40,-3,0))
                .build();
        Trajectory traj2 = robot.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-58,25,0))
                .reverse()
                .strafeLeft(14.5)
                .build();
        Trajectory traj3 = robot.trajectoryBuilder()
                .splineTo(new Pose2d(40,-5,0))
                .addMarker(()->{
                    robot.brat.setPosition(0);
                    return null;
                })
                .forward(90)
                .strafeLeft(27)
                .build();
        Trajectory traj4 = robot.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(-80,-5,0))
                .reverse()
                .back(50)
                .strafeLeft(24)
                .build();
        robot.followTrajectorySync(traj1);
        robot.followTrajectorySync(traj2);
        robot.brat.setPosition(0.1);
        robot.claw.setPosition(0.6);
        sleep(300);
        robot.followTrajectorySync(traj3);
        robot.claw.setPosition(005);
        sleep(300);
        robot.brat.setPosition(0.45);
        robot.followTrajectorySync(traj4);
    }

    private void traiectoriemij(SampleMecanumDriveREVOptimized robot){
        Trajectory traj0 = robot.trajectoryBuilder()
                .back(4)
                .strafeLeft(15)
                .build();
        robot.followTrajectorySync(traj0);
        sleep(200);
        robot.brat.setPosition(0);
        sleep(500);
        robot.claw.setPosition(0);
        sleep(500);
        robot.brat.setPosition(0.6);
        sleep(500);
        robot.claw.setPosition(0.65);

    }

    private String detecteaza(){
        boolean ok = false;
        String x1 = null;
        String x2 = null;
        double px1 = 0;
        double px2 = 0;
        runtime.reset();
        while( runtime.seconds() < 1.5 ){
            if( tfod!=null ){
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == LABEL_SECOND_ELEMENT) ok = true ;
                    }
                    if(updatedRecognitions.size() == 2){
                        x1  = updatedRecognitions.get(0).getLabel();
                        x2  = updatedRecognitions.get(1).getLabel();
                        px1 = updatedRecognitions.get(0).getLeft();
                        px2 = updatedRecognitions.get(1).getLeft();

                    }

                }
            }
        }

        if( !ok ) return "dreapta";
        else{
            if( x2 == LABEL_SECOND_ELEMENT ){
                double aux = px2;
                px2 = px1;
                px1 = aux;
            }
            //acum x1 e skystone

            if( px1 < px2)  return "stanga";
            else return "mijloc";
        }
    }



    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        //tfod.setClippingMargins();
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "camera porno");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}
