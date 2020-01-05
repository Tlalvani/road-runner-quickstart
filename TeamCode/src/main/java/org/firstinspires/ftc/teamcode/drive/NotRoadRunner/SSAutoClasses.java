package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 *
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * LB = Left Back
 * LF = Left Front
 * RB = Right Back
 * RF = Right Front


 */
abstract public class SSAutoClasses extends LinearOpMode {
    //150
    //300
    //550
    //750
    //1040
    //1300

    public static double servorotaterblue = .05;
    public static double servorotatered = .68;
    public static double servorotateback = .35;
    public static double servorotatehome = .93;

    public static double servoarmup = .18;
    public static double servoarmdown = .45;
    public static double servoarmpickup = .5;
    public static double servoarmhome = .37;
    public static double servoarmdeliver = .4;
    public static double servojointup = .23;
    public static double servojointdown =.74;
    public static double servojointdeliver = .6;
    public static double servojointdeliverred = .5;
    public static double servojointhome = .2;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    double dt = 1;
    double maxpower = .48;
    ImprovedSkystoneDetector detector;
    OpenCvCamera webcam;

    protected ElapsedTime runtime = new ElapsedTime();
    /* local OpMode members. */
    HardwareMap hwMap = null;
    protected ElapsedTime period = new ElapsedTime();
    SSHardwareDrivebase robot = new SSHardwareDrivebase();

    private static final String VUFORIA_KEY =
            "AS+iypf/////AAABmVA+Shgi7EdsqH2d8iljwuh6/bNz0HoePOJW6LbOge+udoIv+21ZVTVE7Rbb6pcLmDZS0fP37WoY3OfgCQZBXOdLEpSXhJMHZuyD1V42qLPR9R+FarnuNWS21fr+EhRFPNatPM+riV2eQCS0WroErVmpvwDJUxQCI5Uk9ekS3TPW+9oEf/7V1OUr29wH6lMAwx0SOwTGW37/eaYWOpYpJRPJt9AtLrKupxQc6n9p87o/7oCBifNw0DLLb4L8WN3FaqhHuZ7iWYH3f1D7qcKetbfPW6oOEcTPhIhkERNOn+qiCj8zqJ626Bp1YLtOEEAdP+m25g7D3sJiwW32g/ekIFF8xNTNXwS830fKUHjB2z+D";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /* Constructor */
    public SSAutoClasses() {


    }

    //FUNCTIONS

    public void initSensors() {
        robot.init(hardwareMap);

        robot.AutoArm.setPosition(robot.servoarmup);
        robot.Grab1.setPosition(robot.grab1open);
        robot.Grab2.setPosition(robot.grab2home);
        robot.LeftLatch.setPosition(robot.leftlatchopen);
        robot.RightLatch.setPosition(robot.rightlatchopen);


    }

    //DRIVE FUNCTIONS
    public void Drive(double leftpower, double rightpower) {

        robot.LF.setPower(leftpower);
        robot.RF.setPower(rightpower);
        robot.LB.setPower(leftpower);
        robot.RB.setPower(rightpower);
    }


    public void DriveTargetPosition(int LFpower, int LBpower, int RFpower, int RBpower) {
        robot.LB.setTargetPosition(robot.LB.getCurrentPosition() + LBpower);
        robot.RB.setTargetPosition(robot.RB.getCurrentPosition() + RBpower);
        robot.LF.setTargetPosition(robot.LF.getCurrentPosition() + LFpower);
        robot.RF.setTargetPosition(robot.RF.getCurrentPosition() + RFpower);

        robot.LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void DrivebaseBusy() {
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
telemetry.update();
        }
    }

    public double RampPower(double distance, double maxpower, double minpower) {
        double power = Range.clip(Math.abs(robot.RB.getCurrentPosition() - robot.RB.getTargetPosition()) / distance, minpower, maxpower);
        return power;

    }
    public void RightAngleTurnTargetPosition(int RFpower, int RBpower) {
        robot.RB.setTargetPosition(robot.RB.getCurrentPosition() + RBpower);
        robot.RF.setTargetPosition(robot.RF.getCurrentPosition() + RFpower);

        robot.RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void LeftAngleTurnTargetPosition(int LFpower, int LBpower) {
        robot.LB.setTargetPosition(robot.RB.getCurrentPosition() + LBpower);
        robot.LF.setTargetPosition(robot.RF.getCurrentPosition() + LFpower);

        robot.LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void RightDrivebaseBusy() {
        while (robot.RF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {}
    }

    public void LeftDrivebaseBusy() {
        while (robot.LF.isBusy() & robot.LB.isBusy() & opModeIsActive()) {}
    }


    public void liftup(){
        if(robot.LiftCurrentPosition()<410){
            robot.Lift(.5);
        }
        robot.Lift(0);
    }
    public void ResetArm() {
        robot.AutoArmJoint.setPosition(robot.servojointup);
        robot.AutoArm.setPosition(robot.servoarmup);
        robot.Grab1.setPosition(robot.grab1home);
        robot.Grab2.setPosition(robot.grab2home);
        sleep(2000);
        robot.AutoArmRotate.setPosition(robot.servorotatehome);
        sleep(2000);
        robot.AutoArm.setPosition(robot.servoarmhome);
        robot.AutoArm.setPwmDisable();
        robot.AutoArmJoint.setPwmDisable();
    }

    public void BusyLift() {
        while (robot.Lift1.isBusy() & robot.Lift2.isBusy() /* & robot.Lift3.isBusy()*/ & opModeIsActive()) {}
        robot.Lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.Lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void StonePickup(){
        //Stone pickup
        robot.AutoArm.setPosition(robot.servoarmpickup);
        robot.Grab1.setPosition(robot.grab1open);
        robot.Grab2.setPosition(robot.grab2open);
        robot.AutoArmJoint.setPosition(robot.servojointdown);
        sleep(250);
        robot.Grab1.setPosition(robot.grab1close);
        robot.Grab2.setPosition(robot.grab2close);
        sleep(400);
        robot.AutoArmJoint.setPosition(robot.servojointup);
        robot.AutoArm.setPosition(robot.servoarmdown);
        sleep(200);
    }

    public void StoneDeliver(){
        //Deliver
        robot.AutoArm.setPosition(robot.servoarmdeliver);
        robot.AutoArmJoint.setPosition(robot.servojointdeliver);
        sleep(250);
        robot.Grab1.setPosition(robot.grab1open);
        robot.Grab2.setPosition(robot.grab2open);
        sleep(100);


    }

    protected void initDogeCV() {
        // Start camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();
        this.detector = new ImprovedSkystoneDetector();
        this.detector.useDefaults();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

    }

    protected void initWebcamDogeCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        this.detector = new ImprovedSkystoneDetector();
        this.detector.useDefaults();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

    }

    protected int runDetection(int value) {
        if (detector.getScreenPosition().x < 65) {
            value = 2;
        } else if (detector.getScreenPosition().x < 150) {
            value = 1;
        } else if (detector.getScreenPosition().x < 200) {
            value = 3;
        } else {
            value = 1;
        }
        telemetry.addData("Stone Position X", detector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", detector.getScreenPosition().y);
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
        telemetry.addData("Value", value);
        telemetry.update();

        return value;
    }
    protected int runPhoneDetection(int value) {
        if (detector.getScreenPosition().x < 55) {
            value = 3;
        } else if (detector.getScreenPosition().x < 110) {
            value = 1;
        } else if (detector.getScreenPosition().x < 200) {
            value = 2;
        } else {
            value = 1;
        }
        telemetry.addData("Stone Position X", detector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", detector.getScreenPosition().y);
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
        telemetry.addData("Value", value);
        telemetry.update();

        return value;
    }


    public Vector2d BlueStoneMove(int skystone) {
        Vector2d position = new Vector2d(-60,37);
        if (skystone == 1 || skystone == 0) {
            position = new Vector2d(-60,37);
        } else if (skystone == 2) {
            position = new Vector2d(-52,37);
        } else if (skystone == 3) {
           position = new Vector2d(-44,37);
        }
return position;
    }

    public Vector2d RedStoneMove(int skystone) {
        Vector2d position = new Vector2d(-53,-36);
        if (skystone == 1 || skystone == 0) {
            position = new Vector2d(-53,-36);
        } else if (skystone == 2) {
            position = new Vector2d(-45,-36);
        } else if (skystone == 3) {
            position = new Vector2d(-37,-36);
        }
        return position;
    }

    public void GrabFoundation(){
        robot.LeftLatch.setPosition(robot.leftlatchclose);
        robot.RightLatch.setPosition(robot.rightlatchclose);
    }
    public void ReleaseFoundation(){
        robot.LeftLatch.setPosition(robot.leftlatchopen);
        robot.RightLatch.setPosition(robot.rightlatchopen);
    }

    public void ArmUpReset(){
        robot.AutoArmJoint.setPosition(robot.servojointup);
        robot.AutoArm.setPosition(robot.servoarmup);
        robot.Grab1.setPosition(robot.grab1home);
        robot.Grab2.setPosition(robot.grab2home);
    }

    public void ArmKill(){
        robot.AutoArm.setPosition(robot.servoarmhome);
        robot.AutoArm.setPwmDisable();
        robot.AutoArmJoint.setPwmDisable();
    }
}





