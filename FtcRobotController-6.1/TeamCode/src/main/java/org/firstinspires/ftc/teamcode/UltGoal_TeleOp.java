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

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*/

@TeleOp(name="GoalTeleop", group="14174")
@Disabled
public class UltGoal_TeleOp extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    private static final String VUFORIA_KEY =
            "AQdreXP/////AAABmZt6Oecz+kEzpK0JGPmBsiNN7l/NAvoL0zpZPFQAslTHUcNYg++t82d9o6emZcSfRJM36o491JUmYS/5qdxxP235BssGslVIMSJCT7vNZ2iQW2pwj6Lxtw/oqvCLtgGRPxUyVSC1u5QHi+Siktg3e4g9rYzoQ2+kzv2chS8TnNooSoF6YgQh4FXqCYRizfbYkjVWtx/DtIigXy+TrXNn84yXbl66CnjNy2LFaOdBFrl315+A79dEYJ+Pl0b75dzncQcrt/aulSBllkA4f03FxeN3Ck1cx9twVFatjOCFxPok0OApMyo1kcARcPpemk1mqF2yf2zJORZxF0H+PcRkS2Sv92UpSEq/9v+dYpruj/Vr";
    // Declare OpMode members.

    UltGoal_Hardware robot = new UltGoal_Hardware();

    BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;

    double tX = 0;
    double tY = 0;
    double tZ = 0;

    double rX = 0;
    double rY = 0;
    double rZ = 0;

    double done = 0;
    double step = 0;
    double startTime = getRuntime();
    double step1Time = startTime+10;
    //final double step2Time = startTime+20;
    //final double step3Time = startTime+30;
    double rectifiedRY = ((rY+360)%360)-180;
    double DaS = Math.abs(rectifiedRY);
    double targetLock = 0;

    final double kP = 0.2;
    final double kI = 0.0;
    final double kD = 0;
    final double iLimit = 1;
    double error = 0;

    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
    double dt = 0;
    double errorRate = 0;

    //other motors

    @Override
    public void runOpMode() //throws InterruptedException
    {

        BNO055IMU imu;
        Orientation angles;

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        //Drive motors
        front_left = hardwareMap.get(DcMotor.class, "fl");
        front_right = hardwareMap.get(DcMotor.class, "fr");
        back_left = hardwareMap.get(DcMotor.class, "bl");
        back_right = hardwareMap.get(DcMotor.class, "br");

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);



        robot.init(hardwareMap);
        //Variables

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //waitForStart();
        runtime.reset();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status:", "Waiting for start command.");
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //-----------------------------------------------------------------------------------------
            //-----------------------------------------------------------------------------------------
            // DRIVING CONTROLS
            //-----------------------------------------------------------------------------------------
            //-----------------------------------------------------------------------------------------

            forward = -gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            clockwise = gamepad1.right_stick_x;
            front_left.setPower(0.25 * Range.clip(forward + clockwise + right, -1, 1));
            front_right.setPower(0.25 * Range.clip(forward - clockwise - right, -1, 1));
            back_left.setPower(0.25 * Range.clip(forward + clockwise - right, -1, 1));
            back_right.setPower(0.25 * Range.clip(forward - clockwise + right, -1, 1));
            
            if (Math.abs(gamepad1.left_stick_y) > 0.01) {
                if (gamepad1.left_stick_y > 0) {
                    forward = -(gamepad1.left_stick_y * gamepad1.left_stick_y);
                } else if (gamepad1.left_stick_y < 0) {
                    forward = (gamepad1.left_stick_y * gamepad1.left_stick_y);
                }
            } else {
                forward = 0;
            }

            if (Math.abs(gamepad1.left_stick_x) > 0.01) {
                if (gamepad1.left_stick_x > 0) {
                    right = gamepad1.left_stick_x * gamepad1.left_stick_x;
                } else if (gamepad1.left_stick_x < 0) {
                    right = -(gamepad1.left_stick_x * gamepad1.left_stick_x);
                }
            } else {
                right = 0;
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.01) {
                if (gamepad1.right_stick_x > 0) {
                    clockwise = gamepad1.right_stick_x * gamepad1.right_stick_x;
                } else if (gamepad1.right_stick_x < 0) {
                    clockwise = -(gamepad1.right_stick_x * gamepad1.right_stick_x);
                }
            } else {
                clockwise = 0;
            }

            if (gamepad1.right_trigger > 0.1) {
                front_left.setPower(0.25 * Range.clip(forward + clockwise + right, -1, 1));
                front_right.setPower(0.25 * Range.clip(forward - clockwise - right, -1, 1));
                back_left.setPower(0.25 * Range.clip(forward + clockwise - right, -1, 1));
                back_right.setPower(0.25 * Range.clip(forward - clockwise + right, -1, 1));
            } else {
                front_left.setPower(Range.clip(forward + clockwise + right, -1, 1));
                front_right.setPower(Range.clip(forward - clockwise - right, -1, 1));
                back_left.setPower(Range.clip(forward + clockwise - right, -1, 1));
                back_right.setPower(Range.clip(forward - clockwise + right, -1, 1));
            }



            //DRIVETRAIN



        }

    }
}
