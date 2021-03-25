package org.firstinspires.ftc.teamcode.drive.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.UltGoal_Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;

import java.util.List;

@Autonomous(name = "RedOuter", group = "14174")
public class redOuter extends LinearOpMode {
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
        robot.aim.setPosition(robot.aimGoal);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 1);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-63, -48.5, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        drive.update();

        waitForStart();

        if (isStopRequested()) return;
        //UNIVERSAL TRAJECTORIES



        Trajectory trajpark = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(6, -48))
                .build();



        //NO RING TRAJECTORIES
        Trajectory traj01 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-34, -55, Math.toRadians(0))) //move towards the outside of the field to avoid the other robot
                .splineToSplineHeading(new Pose2d(-3, -55, Math.toRadians(-90)), Math.toRadians(0)) //move forward and spin around to drop off the wobble goal
                .addDisplacementMarker(() -> {
                    robot.armOut.setPosition(robot.armDown); //drop the wobble goal arm
                })
                .build();
        //robot.pincher.setPosition(robot.unPinched);

        Trajectory traj02 = drive.trajectoryBuilder(traj01.end(), true)
                .addDisplacementMarker(1, () -> {
                    robot.armOut.setPosition(robot.armUp);
                })
                .addDisplacementMarker(40, () -> {
                    robot.flyWheel.setPower(0.95);
                })
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory traj03 = drive.trajectoryBuilder(traj02.end())
                .addDisplacementMarker(8, () -> {
                    robot.flyWheel.setPower(0.97);
                })
                .splineToLinearHeading(new Pose2d(-6, -55, Math.toRadians(15)), Math.toRadians(0)) //move behind the white line and shoot for the white
                .build();

        //robot.kicker.setPosition(robot.kickerOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickerIn);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickerOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickerIn);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickerOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickerIn);

        Trajectory traj04 = drive.trajectoryBuilder(traj03.end())
                .addDisplacementMarker(() -> {
                    robot.flyWheel.setPower(0);
                })
                .lineTo(new Vector2d(14, -45))
                .build();

        //SINGLE STACK TRAJECTORIES

        Trajectory traj11 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-34, -55, Math.toRadians(0))) //move towards the outside of the field to avoid the other robot
                .splineToSplineHeading(new Pose2d(40, -55, Math.toRadians(0)), Math.toRadians(0)) //move forward and spin around to drop off the wobble goal
                .addDisplacementMarker(() -> {
                    robot.armOut.setPosition(robot.armDown); //drop the wobble goal arm
                })
                .build();
        //robot.pincher.setPosition(robot.unPinched);

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end(),  true) //the robot will be driving backwards
                .addDisplacementMarker(1, () -> {
                    robot.armOut.setPosition(robot.armUp);
                })
                .addDisplacementMarker(40, () -> {
                    robot.flyWheel.setPower(0.95);
                })
                .splineToLinearHeading(new Pose2d(-6, -55, Math.toRadians(-15)), Math.toRadians(0)) //move behind the white line and shoot for the white
                .build();

        //robot.kicker.setPosition(robot.kickOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickIn);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickIn);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickIn);
        //sleep(500);

        Trajectory traj13 = drive.trajectoryBuilder(traj12.end())
                .addDisplacementMarker(() -> {
                    robot.flyWheel.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(14,  -55, Math.toRadians(0))) //park on white line
                .build();

        //QUAD STACK TRAJECTORIES
        Trajectory traj41 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-34, -55, Math.toRadians(0))) //move towards the outside of the field to avoid the other robot
                .splineToSplineHeading(new Pose2d(51, -55, Math.toRadians(-90)), Math.toRadians(0)) //move forward and spin around to drop off the wobble goal
                .addDisplacementMarker(() -> {
                    robot.armOut.setPosition(robot.armDown); //drop the wobble goal arm
                })
                .build();
        //robot.pincher.setPosition(robot.unPinched);

        Trajectory traj42 = drive.trajectoryBuilder(traj41.end(),  true) //the robot will be driving backwards
                .addDisplacementMarker(1, () -> {
                    robot.armOut.setPosition(robot.armUp);
                })
                .addDisplacementMarker(40, () -> {
                    robot.flyWheel.setPower(0.95);
                })
                //.splineTo(new Vector2d(0, 0), Math.toRadians(0)) //move toward blue wall
                .splineToLinearHeading(new Pose2d(-6, -55, Math.toRadians(-15)), Math.toRadians(0)) //move behind the white line and shoot for the white
                .build();

        //robot.kicker.setPosition(robot.kickOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickIn);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickIn);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickOut);
        //sleep(500);
        //robot.kicker.setPosition(robot.kickIn);
        //sleep(500);

        Trajectory traj43 = drive.trajectoryBuilder(traj42.end())
                .addDisplacementMarker(() -> {
                    robot.flyWheel.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(14,  -55, Math.toRadians(0))) //park on white line
                .build();


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
            drive.followTrajectory(traj01);
            robot.pincher.setPosition(robot.unPinched);
            sleep(500);
            drive.followTrajectory(traj02);
            sleep(5000);
            drive.followTrajectory(traj03);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            sleep(500);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            sleep(500);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            drive.followTrajectory(traj04);
            PoseStorage.currentPose = drive.getPoseEstimate();
            stop();
        }
        else if (ringState == 1) {
            drive.followTrajectory(traj11);
            robot.pincher.setPosition(robot.unPinched);
            sleep(500);
            drive.followTrajectory(traj12);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            sleep(500);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            sleep(500);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            sleep(500);
            drive.followTrajectory(traj13);
            PoseStorage.currentPose = drive.getPoseEstimate();
            stop();
        }
        else if(ringState == 4) {
            drive.followTrajectory(traj41);
            robot.pincher.setPosition(robot.unPinched);
            sleep(500);
            drive.followTrajectory(traj42);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            sleep(500);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            sleep(500);
            robot.kicker.setPosition(robot.kickerOut);
            sleep(500);
            robot.kicker.setPosition(robot.kickerIn);
            sleep(500);
            drive.followTrajectory(traj43);
            PoseStorage.currentPose = drive.getPoseEstimate();
            stop();
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