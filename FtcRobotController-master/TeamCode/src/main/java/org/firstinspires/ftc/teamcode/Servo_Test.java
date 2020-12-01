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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Servo Test", group="14174")
@Disabled
public class Servo_Test extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Servo Servo1;
    //Servo Servo2;
    //CRServo CRServo1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //Servo1 = hardwareMap.servo.get("cap");
        //Servo1 = hardwareMap.servo.get("grabber");
        Servo1 = hardwareMap.servo.get("cap2");
        //Servo2 = hardwareMap.servo.get("pincher");
        //Servo2 = hardwareMap.servo.get("foundationL");
        //CRServo1 = hardwareMap.crservo.get("slide");

        // Most robots need the motor on one side to` be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //Servo1.setPosition(0.7);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //Servo2.setDirection(Servo.Direction.FORWARD);
        Servo1.setDirection(Servo.Direction.FORWARD);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                Servo1.setPosition(Servo1.getPosition() + .001);
                telemetry.addData("Servo1 Position", Servo1.getPosition());
                telemetry.addData("Direction", Servo1.getDirection());
                telemetry.update();
            }
            if (gamepad1.b) {
                Servo1.setPosition(Servo1.getPosition() - .001);
                telemetry.addData("Servo1 Position", Servo1.getPosition());
                telemetry.addData("Direction", Servo1.getDirection());
                telemetry.update();
            }


            /*if(gamepad1.x) {
                Servo2.setPosition(Servo2.getPosition() + .001);
                telemetry.addData("Servo2 Position", Servo2.getPosition());
                telemetry.addData("Direction", Servo2.getDirection());
                telemetry.update();
            }
            if(gamepad1.y) {
                Servo2.setPosition(Servo2.getPosition() - .001);
                telemetry.addData("Servo2 Position", Servo2.getPosition());
                telemetry.addData("Direction", Servo2.getDirection());
                telemetry.update();
            }*/

            /*
            if(gamepad1.a) {
                CRServo1.setPower(1);
            } else if(gamepad1.b) {
                CRServo1.setPower(-1);
            } else {
                CRServo1.setPower(0);
            }
             */
        }
    }
}