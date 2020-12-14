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
    public DcMotor transfer;
    public DcMotor arm;

    public Servo kicker;
    public Servo aim;
    public Servo pincher;
    public Servo lock;
    public Servo kickOut;
    public Servo armOut;


    //DEFINE SERVOS

    public final double height = 26;
    public final double kickerIn = 0.5757;
    public final double kickerOut = 0.3;
    public final double powerAngle = 0;
    public final double aimMax = 0.663;
    public final double aimMin = 0.4848;
    public final double aimInit = 0.6;
    public final double collectAngle = -18.81;
    public final double lockDistance = 0.31; //0.17 is in, 0.85 is out
    public final double kickOutDistance = 0;

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
        transfer = hwMap.get(DcMotor.class, "transfer");
        arm = hwMap.get(DcMotor.class, "arm");

        aim = ahwMap.servo.get("aim");
        kicker = ahwMap.servo.get("kicker");
        pincher = ahwMap.servo.get("pincher");
        lock = ahwMap.servo.get("lock");
        kickOut = ahwMap.servo.get("kickOut");
        armOut = ahwMap.servo.get("armOut");

        aim.setDirection(Servo.Direction.REVERSE);

        //Auto
        kicker.setPosition(kickerIn);
        lock.setPosition(0.17);
        kickOut.setPosition(kickOutDistance);

        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collection.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //robot.init(hardwareMap);

        // Define all Servos
        aim.setPosition(aimInit);

        //Inititialize Servos

    }
}