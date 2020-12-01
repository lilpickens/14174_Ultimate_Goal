package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class UltGoal_Hardware
{
    /* Public OpMode members. */
    //DEFINE MOTORS
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;

    public DcMotor flyWheel;

    public Servo kicker;
    public Servo aim;

    //DEFINE SERVOS

    public final double height = 26;
    public final double kickerIn = 0;
    public final double kickerOut = 0;
    public final double powerAngle = 0;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public UltGoal_Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        flyWheel = hwMap.get(DcMotor.class, "flywheel");

        aim = ahwMap.servo.get("aim");
        kicker = ahwMap.servo.get("kicker");

        //Auto


        //robot.init(hardwareMap);

        // Define all Servos

        //Inititialize Servos

    }
}