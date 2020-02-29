package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

/**
 * This is NOT an opmode.
 * <p>
 * <p>
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * LB = Left Back
 * LF = Left Front
 * RB = Right Back
 * RF = Right Front
 */
public class SSHardwareDrivebase {

    double divisorforimu = 150;
    double maxspeedimu = .25;
    double pdmaxspeedimu = .8;
    double minspeedimu = 0;
    double currentangle = 0;
    double AngleTolerance = 3.5;
 double maxpower = .3;

//leftclaw is gripper
    //right claw is capstone

    double leftclawclose = .9; //.72
    double rightclawclose = .2; //.27
    double leftclawopen = .49;
    double gripperup = .2;
   double rightclawopen = .8;

    double leftintakearmout = .5; //.52
    double rightintakearmout = .5; //.56

    double leftintakearmfat = .3; //.52
    double rightintakearmfat = .65; //.56

    double leftintakearmin = 1;
    double rightintakearmin = .35;

    double servorotaterblue = .12;
    double servorotatered = .73;

    double servorotateback = .41;
    double servorotatehome = .98;

    double servorotateredpartial = .57;
    double servorotatebluepartial = .49;


    double servoarmup = .41;    //.18
    double servoarmdown = .68;  //.43
   double servoarmpickup = .77; //.49
    double servoarmhigh = .25;
    double servoarmhome = .37;
    double servoarmdeliver = .65;

    double servojointup = .18;
    double servojointdown =.7;
    double servojointdeliver = .6;
    double servojointdeliverred = .5;
    double servojointhome = .2;

    double leftlatchopen = .35;
    double rightlatchopen = .7;

    double leftlatchclose = .5; //.6
    double rightlatchclose = .45; //.45

    double grab1close = .7;
    double grab2close = .20;

    double grab1open = .3;
    double grab2open = .6;

    double grab1home = .5;
    double grab2home = .8;

double parkslidein = .75;
double parkslideout = .5;
    boolean Liftedup = false;



    /* Public OpMode members. */
    public DcMotor Lift1, Lift2, LeftIntake, RightIntake;
    public Servo LeftLatch, RightLatch, LeftClaw, RightClaw, LeftIntakeArm, RightIntakeArm, Grab1, Grab2,ParkSlide;
    public ServoImplEx AutoArm,AutoArmJoint,AutoArmRotate;
public DcMotorEx PID;
    public DcMotorEx LF,RF,LB,RB;


    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public SSHardwareDrivebase() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LB = hwMap.get(DcMotorEx.class, "LB");
        RB = hwMap.get(DcMotorEx.class, "RB");
        PID = hwMap.get(DcMotorEx.class, "RB");
        RF = hwMap.get(DcMotorEx.class, "RF");
        LF = hwMap.get(DcMotorEx.class, "LF");
        Lift1 = hwMap.dcMotor.get("Lift1");
        Lift2 = hwMap.dcMotor.get("Lift2");
        LeftIntake = hwMap.dcMotor.get("leftintake");
        RightIntake = hwMap.dcMotor.get("rightintake");

        LeftLatch = hwMap.servo.get("leftlatch");
        RightLatch = hwMap.servo.get("rightlatch");

        LeftClaw = hwMap.servo.get("leftclaw");
        RightClaw = hwMap.servo.get("rightclaw");

        LeftIntakeArm = hwMap.servo.get("leftintakearm");
        RightIntakeArm = hwMap.servo.get("rightintakearm");

        AutoArm =  hwMap.get(ServoImplEx.class, "autoarm");
        AutoArmJoint =  hwMap.get(ServoImplEx.class, "autoarmjoint");
        AutoArmRotate =  hwMap.get(ServoImplEx.class, "autoarmrotate");
        Grab1 = hwMap.servo.get("grab1");
        Grab2 = hwMap.servo.get("grab2");

        ParkSlide = hwMap.servo.get("parkslide");


        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        Lift2.setDirection(DcMotor.Direction.FORWARD);
        Lift1.setDirection(DcMotor.Direction.REVERSE);

        RightIntake.setDirection(DcMotor.Direction.FORWARD);
        LeftIntake.setDirection(DcMotor.Direction.FORWARD);



        // Set all motors to zero power
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        LF.setPower(0);
        Lift1.setPower(0);
        Lift2.setPower(0);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    public void Lift(double power) {
        Lift1.setPower(power);
        Lift2.setPower(power);
       // Lift3.setPower(power);
    }

    public void LiftPosition(int liftposition) {
        Lift1.setTargetPosition(liftposition);
        Lift2.setTargetPosition(liftposition);
        Lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void LiftWithEncoders() {
        Lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public int LiftCurrentPosition() {
        int position = (Lift1.getCurrentPosition() + Lift2.getCurrentPosition()/2);
        return position;
    }



    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }


    public void DrivebaseWithEncoders() {
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ParkSlideIn(){
        ParkSlide.setPosition(parkslidein);
    }
    public void ParkSlideOut(){
        ParkSlide.setPosition(parkslideout);
    }

}






