package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;

import org.firstinspires.ftc.teamcode.UltGoal_Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import java.util.Vector;

@Autonomous(name = "Blue_Auto", group = "14174")
public class RRTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    UltGoal_Hardware robot = new UltGoal_Hardware();

    private static final String VUFORIA_KEY =
            "AQdreXP/////AAABmZt6Oecz+kEzpK0JGPmBsiNN7l/NAvoL0zpZPFQAslTHUcNYg++t82d9o6emZcSfRJM36o491JUmYS/5qdxxP235BssGslVIMSJCT7vNZ2iQW2pwj6Lxtw/oqvCLtgGRPxUyVSC1u5QHi+Siktg3e4g9rYzoQ2+kzv2chS8TnNooSoF6YgQh4FXqCYRizfbYkjVWtx/DtIigXy+TrXNn84yXbl66CnjNy2LFaOdBFrl315+A79dEYJ+Pl0b75dzncQcrt/aulSBllkA4f03FxeN3Ck1cx9twVFatjOCFxPok0OApMyo1kcARcPpemk1mqF2yf2zJORZxF0H+PcRkS2Sv92UpSEq/9v+dYpruj/Vr";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int ringState = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.pincher.setPosition(robot.pinched);
        robot.armOut.setPosition(robot.armUp);
        robot.kicker.setPosition(robot.kickerIn);
        robot.aim.setPosition(robot.aimPower + 0.02);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 1);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-63, 25, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        drive.update();

        waitForStart();

        if (isStopRequested()) return;
        //UNIVERSAL TRAJECTORIES



        Trajectory trajpark = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(6, 48))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(startPose /*traj0.end()*/)
                .addDisplacementMarker(() -> {
                    robot.flyWheel.setPower(0.79 * ((12.9/drive.batteryVoltageSensor.getVoltage())));
                })
                .splineToLinearHeading(new Pose2d(-11,14, Math.toRadians(-1)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.kicker.setPosition(robot.kickerOut);
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .addDisplacementMarker(() -> {
                    robot.kicker.setPosition(robot.kickerIn);
                })
                .lineToConstantHeading(new Vector2d(-10.5, 20.5))
                .addDisplacementMarker(() -> {
                    robot.kicker.setPosition(robot.kickerOut);
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .addDisplacementMarker(() -> {
                    robot.kicker.setPosition(robot.kickerIn);
                })
                .lineToConstantHeading(new Vector2d(-10, 28.5)) //24.75 //27.75 //29
                .addDisplacementMarker(() -> {
                    robot.kicker.setPosition(robot.kickerOut);
                })
                .build();

        //NO RING TRAJECTORIES
        Trajectory traj04 = drive.trajectoryBuilder(traj3.end())
                .addDisplacementMarker(() -> {
                    robot.flyWheel.setPower(0);

                    robot.kicker.setPosition(robot.kickerIn);
                })
                .lineToLinearHeading(new Pose2d(18, 54, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    robot.armOut.setPosition(robot.armDown);
                    robot.pincher.setPosition(robot.unPinched);
                })
                .build();

        Trajectory traj05 = drive.trajectoryBuilder(traj04.end(), true)
                .addDisplacementMarker(() -> {
                    robot.armOut.setPosition(robot.armUp);
                })
                .splineToLinearHeading(new Pose2d(-53, 33, Math.toRadians(0)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    robot.armOut.setPosition(robot.armDown);
                    robot.pincher.setPosition(robot.unPinched);
                })
                .build();

        Trajectory traj06 = drive.trajectoryBuilder(traj05.end())
                .addDisplacementMarker(4, () -> {
                    robot.pincher.setPosition(robot.pinched);
                })
                .splineToConstantHeading(new Vector2d(12, 48), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.pincher.setPosition(robot.unPinched);
                })
                .build();

        Trajectory traj0park = drive.trajectoryBuilder(traj06.end())
                
                .lineTo(new Vector2d(12, 30))
                .build();

        //SINGLE STACK TRAJECTORIES
        Trajectory traj14 = drive.trajectoryBuilder(traj3.end())
                .addDisplacementMarker(() -> {
                    robot.flyWheel.setPower(0);
                    robot.kicker.setPosition(robot.kickerIn);
                })
                .splineTo(new Vector2d(36, 18), Math.toRadians(0))
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj14.end())
                .lineTo(new Vector2d(10, 18))
                .build();

        /*
        Trajectory traj14 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(30, 24), Math.toRadians(0))
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj14.end())
                .lineTo(new Vector2d(0, 15))
                .splineToConstantHeading(new Vector2d(-65, 21), Math.toRadians(180))
                .build();
        Trajectory traj16 = drive.trajectoryBuilder(traj15.end())
                .lineTo(new Vector2d(-65, 50))
                .build();
        Trajectory traj17 = drive.trajectoryBuilder(traj16.end())
                .splineTo(new Vector2d(-10,  50), Math.toRadians(0))
                .splineTo(new Vector2d(30, 24), Math.toRadians(0))
                .build();
        Trajectory traj18 = drive.trajectoryBuilder(traj17.end())
                .lineTo(new Vector2d(6, 24))
                .build();
        */

        /*
        Trajectory traj14 = drive.trajectoryBuilder(traj0.end())
                .splineTo(new Vector2d(42, 25), Math.toRadians(0))
                .build();
        Trajectory traj15 = drive.trajectoryBuilder(traj14.end())
                .splineToSplineHeading(new Pose2d(11, 23, Math.toRadians(180)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-15, 35), Math.toRadians(180))
                .build();
        
        Trajectory traj16 = drive.trajectoryBuilder(traj15.end())
                .splineToLinearHeading(new Pose2d(-65, 32, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-65, 42), Math.toRadians(90))
                .build();
        Trajectory traj17 = drive.trajectoryBuilder(traj16.end())
                .splineTo(new Vector2d(-3, 36), Math.toRadians(0))
                .build();
        Trajectory traj18 = drive.trajectoryBuilder(traj17.end())
                .splineTo(new Vector2d(36, 25), Math.toRadians(0))
                .build();
        Trajectory traj19 = drive.trajectoryBuilder(traj18.end())
                .lineTo(new Vector2d(6, 25))
                .build();

        //QUAD STACK TRAJECTORIES

         */
        Trajectory traj44 = drive.trajectoryBuilder(traj3.end())
                .addDisplacementMarker(() -> {
                    robot.flyWheel.setPower(0);
                    robot.kicker.setPosition(robot.kickerIn);
                })
                .splineTo(new Vector2d(60, 48), Math.toRadians(0))
                .build();
        Trajectory traj45 = drive.trajectoryBuilder(traj44.end())
                .lineTo(new Vector2d(10, 48))
                .build();
        /*
        Trajectory traj44 = drive.trajectoryBuilder(traj0.end())
                .splineTo(new Vector2d(46, 47), Math.toRadians(0))
                .build();
        Trajectory traj45 = drive.trajectoryBuilder(traj44.end())
                .lineTo(new Vector2d(0, 15))
                .splineToConstantHeading(new Vector2d(-65, 21), Math.toRadians(180))
                .build();
        Trajectory traj46 = drive.trajectoryBuilder(traj45.end())
                .lineTo(new Vector2d(-65, 50))
                .build();
        Trajectory traj47 = drive.trajectoryBuilder(traj46.end())
                .splineTo(new Vector2d(46,  47), Math.toRadians(0))
                .build();
        Trajectory traj48 = drive.trajectoryBuilder(traj47.end())
                .lineTo(new Vector2d(6, 47))
                .build();

         */

        
        //new 0 STACK TRAJECTORY
        /*
        Trajectory traj01 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-52, 16), Math.toRadians(0))
                .build();
        Trajectory traj02 = drive.trajectoryBuilder(traj01.end())
                .splineToConstantHeading(new Vector2d(48, 58), Math.toRadians(0))
                .build();
        Trajectory traj03 = drive.trajectoryBuilder(traj02.end())
                .splineToConstantHeading(new Vector2d(12, 40), Math.toRadians(0))
                .build();
        
        Trajectory traj11 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-52, 16), Math.toRadians(0))
                .build();
        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .splineToConstantHeading(new Vector2d(-24, 58), Math.toRadians(0))
                .build();
        Trajectory traj13 = drive.trajectoryBuilder(traj12.end())
                .splineToConstantHeading(new Vector2d(12, 16), Math.toRadians(0))
                .build();
        
        Trajectory traj41 = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-52, 16), Math.toRadians(0))
                .build();
        Trajectory traj42 = drive.trajectoryBuilder(traj41.end())
                .splineToConstantHeading(new Vector2d(-8, 16), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-58, 58), Math.toRadians(0))
                .build();
        Trajectory traj43 = drive.trajectoryBuilder(traj42.end())
                .splineToConstantHeading(new Vector2d(12, 52), Math.toRadians(0))
                .build();
         */

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0 ) {
                    // empty list.  no objects recognized.
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("Target Zone", "A");
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        // check label to see which target zone to go after.
                        if (recognition.getLabel().equals("Single")) {
                            telemetry.addData("Target Zone", "B");
                            ringState = 1;
                        } else if (recognition.getLabel().equals("Quad")) {
                            telemetry.addData("Target Zone", "C");
                            ringState = 4;
                        } else {
                            telemetry.addData("Target Zone", "UNKNOWN");
                            ringState = 0;
                        }
                    }
                }

                telemetry.update();
            }
        }

        tfod.deactivate();

        if (ringState == 0) {
            drive.followTrajectory(traj1);
            sleep(500);
            drive.followTrajectory(traj2);
            sleep(500);
            drive.followTrajectory(traj3);
            sleep(500);
            drive.followTrajectory(traj04);
            sleep(500);
            drive.followTrajectory(traj05);
            sleep(500);
            drive.followTrajectory(traj06);
            sleep(500);
            drive.followTrajectory(traj0park);
            PoseStorage.currentPose = drive.getPoseEstimate();
            stop();
        }
        else if (ringState == 1) {
            //drive.followTrajectory(trajpark);
            drive.followTrajectory(traj1);
            //robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            drive.followTrajectory(traj2);
            //robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            drive.followTrajectory(traj3);
            //robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.armOut.setPosition(robot.armDown);
            drive.followTrajectory(traj14);
            robot.pincher.setPosition(robot.unPinched);
            sleep(500);
            drive.followTrajectory(traj15);
            PoseStorage.currentPose = drive.getPoseEstimate();
            stop();
            /*
            drive.followTrajectory(traj0);
            sleep(200);
            //drive.followTrajectory(traj1);
            //sleep(200);
            //drive.followTrajectory(traj2);
            //sleep(200);
            //drive.followTrajectory(traj3);
            //sleep(200);
            drive.followTrajectory(traj14);
            sleep(200);
            drive.followTrajectory(traj15);
            drive.followTrajectory(traj16);
            sleep(200);
            drive.followTrajectory(traj17);
            sleep(200);
            drive.followTrajectory(traj18);
            PoseStorage.currentPose = drive.getPoseEstimate();
            stop();

            /*
            drive.followTrajectory(traj11);
            robot.pincher.setPosition(robot.pinched);
            sleep(2000);
            drive.followTrajectory(traj12);
            robot.pincher.setPosition(robot.unPinched);
            sleep(2000);
            drive.followTrajectory(traj13);
             */
        }
        else if(ringState == 4) {
            //drive.followTrajectory(trajpark);
            drive.followTrajectory(traj1);
            //robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            drive.followTrajectory(traj2);
            //robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            drive.followTrajectory(traj3);
            //robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.armOut.setPosition(robot.armDown);
            drive.followTrajectory(traj44);
            robot.pincher.setPosition(robot.unPinched);
            sleep(500);
            drive.followTrajectory(traj45);
            PoseStorage.currentPose = drive.getPoseEstimate();
            stop();
            /*
            drive.followTrajectory(traj0);
            sleep(200);
            //drive.followTrajectory(traj1);
            //sleep(200);
            //drive.followTrajectory(traj2);
            //sleep(200);
            //drive.followTrajectory(traj3);
            //sleep(200);
            drive.followTrajectory(traj44);
            sleep(200);
            drive.followTrajectory(traj45);
            drive.followTrajectory(traj46);
            sleep(200);
            drive.followTrajectory(traj47);
            sleep(200);
            drive.followTrajectory(traj48);
            PoseStorage.currentPose = drive.getPoseEstimate();
            stop();

            /*
            drive.followTrajectory(traj41);
            robot.pincher.setPosition(robot.pinched);
            sleep(2000);
            drive.followTrajectory(traj42);
            robot.pincher.setPosition(robot.unPinched);
            sleep(2000);
            drive.followTrajectory(traj43);
             */
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}