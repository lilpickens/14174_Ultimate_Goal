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

    public DcMotor flyWheel;
    public DcMotor collection;
    public DcMotor arm;

    public Servo kicker;
    public Servo aim;
    public Servo pincher;

    //DEFINE SERVOS

    public final double height = 26;
    public final double kickerIn = 0;
    public final double kickerOut = 0;
    public final double powerAngle = 0;
    public final double aimMax = 1;
    public final double aimMin = 0;
    public final double collectAngle = 0;

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
        collection = hwMap.get(DcMotor.class, "collection");
        arm = hwMap.get(DcMotor.class, "arm");

        aim = ahwMap.servo.get("aim");
        kicker = ahwMap.servo.get("kicker");
        pincher = ahwMap.servo.get("pincher");

        //Auto


        //robot.init(hardwareMap);

        // Define all Servos

        //Inititialize Servos

    }
}