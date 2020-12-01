package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.UltGoal_Hardware;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp(name = "TeleOp Blue", group = "14174")
public class AugmentedTeleOp extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    private Vector2d targetPosition = new Vector2d(71, 36);

    double powerShotMode = 0;
    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(-5, 33);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(0);

    double lastShot = 0;
    Pose2d targetPower1 = new Pose2d(-5, 20, Math.toRadians(0));
    Pose2d targetPower2 = new Pose2d(-5, 12.5, Math.toRadians(0));
    Pose2d targetPower3 = new Pose2d(-5, 5, Math.toRadians(0));

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-5, 17);

    double angle = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        UltGoal_Hardware robot = new UltGoal_Hardware();
        headingController.setInputBounds(-Math.PI, Math.PI);
        Pose2d driveDirection = new Pose2d();


        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        robot.init(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status:", "Waiting for start command.");
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );
                    drive.setWeightedDrivePower(driveDirection);

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        //Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                        //        .lineToLinearHeading(targetA)
                        //        .build();

                        //drive.followTrajectoryAsync(traj1);

                        //targetAHeading = Math.atan((136-(drive.getPoseEstimate().getY()+100))/(171-(drive.getPoseEstimate().getX()+100)));
                        //Pose2d traj1pose = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), targetAHeading);
                        //Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                        //        .lineToLinearHeading(traj1pose)
                        //        .build();

                        Vector2d fieldFrameInput = new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        );
                        Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                        // Difference between the target vector and the bot's position
                        Vector2d difference = targetPosition.minus(poseEstimate.vec());
                        // Obtain the target angle for feedback and derivative for feedforward
                        double theta = difference.angle();

                        // Not technically omega because its power. This is the derivative of atan2
                        double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                        // Set the target heading for the heading controller to our desired angle
                        headingController.setTargetPosition(theta);

                        // Set desired angular velocity to the heading controller output + angular
                        // velocity feedforward
                        double headingInput = (headingController.update(poseEstimate.getHeading())
                                * DriveConstants.kV + thetaFF)
                                * DriveConstants.TRACK_WIDTH;

                        // Combine the field centric x/y velocity with our derived angular velocity
                        driveDirection = new Pose2d(
                                robotFrameInput,
                                headingInput
                        );

                        drive.setWeightedDrivePower(driveDirection);

                        currentMode = Mode.AUTOMATIC_CONTROL;

                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(targetPower1)
                                .build();
                        Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(targetPower2)
                                .build();
                        Trajectory traj3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(targetPower3)
                                .build();

                        if (powerShotMode == 0 && getRuntime() > lastShot+0.2) {drive.followTrajectoryAsync(traj1); powerShotMode = 1; lastShot = getRuntime();}
                        else if (powerShotMode == 1 && getRuntime() > lastShot+0.2) {drive.followTrajectoryAsync(traj2); powerShotMode = 2; lastShot = getRuntime();}
                        else if (powerShotMode == 2 && getRuntime() > lastShot+0.2) {drive.followTrajectoryAsync(traj3); powerShotMode = 0; lastShot = getRuntime();}

                        currentMode = Mode.AUTOMATIC_CONTROL;

                    } /* else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } */
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
            //autoaim
            angle = Math.atan(robot.height/Math.sqrt((poseEstimate.getX()*poseEstimate.getX())+(poseEstimate.getY()*poseEstimate.getY())));
            robot.aim.setPosition(angle/180);

            if (gamepad2.right_trigger > 0.1) {robot.flyWheel.setPower(1);}
            else {robot.flyWheel.setPower(0);}
            if (gamepad2.a) {}//collection goes here
            else {}
            if (gamepad2.x) {}//collection reverse goes here
            else {}

            if (gamepad2.b) {}
        }
    }
}