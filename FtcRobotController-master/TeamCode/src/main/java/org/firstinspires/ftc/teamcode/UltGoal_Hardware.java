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
    ModernRoboticsI2cRangeSensor range;
    //DEFINE MOTORS
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;

    //DEFINE SERVOS
    //public Servo foundation_New1;

    //public CRServo intakeCollector;

    //SERVO DEFINED VALUES
    //public static final double intakeUp = 0.53;
    public static final double wheelBase = 0;

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
        //Drive motors
        front_left = hwMap.get(DcMotor.class, "fl");
        front_right = hwMap.get(DcMotor.class, "fr");
        back_left = hwMap.get(DcMotor.class, "bl");
        back_right = hwMap.get(DcMotor.class, "br");

        //foundation_New2 = hwMap.servo.get("foundation2New");
        //lift = hwMap.dcMotor.get("lift");

        //Auto

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //robot.init(hardwareMap);
        //capstone2.setPosition(cap2In);



        // Define all Servos

        //Inititialize Servos

    }
}