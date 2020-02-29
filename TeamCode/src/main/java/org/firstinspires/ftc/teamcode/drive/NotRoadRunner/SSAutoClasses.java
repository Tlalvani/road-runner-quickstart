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
    SSHardwareDrivebase robot = new SSHardwareDrivebase();

    public static double servorotaterblue = new SSHardwareDrivebase().servorotaterblue;
    public static double servorotatered = new SSHardwareDrivebase().servorotatered;
    public static double servorotateinvertblue = servorotatered+.02;
    public static double servorotateinvertred= servorotaterblue-.04;    //negative goes counter clockwise relative to front of robot

    public static double servorotateback = new SSHardwareDrivebase().servorotateback;
    public static double servorotatehome = new SSHardwareDrivebase().servorotatehome;

    public static double servorotateredpartial = new SSHardwareDrivebase().servorotateredpartial;

    public static double servorotatebluepartial = new SSHardwareDrivebase().servorotatebluepartial;


    public static double servoarmup = new SSHardwareDrivebase().servoarmup;
    public static double servoarmdown = new SSHardwareDrivebase().servoarmdown;
    public static double servoarmpickup = new SSHardwareDrivebase().servoarmpickup;
    public static double servoarmhome = new SSHardwareDrivebase().servoarmhome;
    public static double servoarmdeliver = new SSHardwareDrivebase().servoarmdeliver;
    public static double servoarmhigh = new SSHardwareDrivebase().servoarmhigh;
    public static double servoarmbridge = .74;

    public static double grab1open = new SSHardwareDrivebase().grab1open;
    public static double grab2open = new SSHardwareDrivebase().grab2open;

    public static double servojointback = .15;
    public static double servojointup = .2;
    public static double servojointdown =.7;
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
    ImprovedSkystonePhoneDetector phonedetector;
    OpenCvCamera webcam;

    protected ElapsedTime runtime = new ElapsedTime();
    /* local OpMode members. */
    HardwareMap hwMap = null;
    protected ElapsedTime period = new ElapsedTime();


    public double grab1close = robot.grab1close;
    public double grab2close = robot.grab2close;

    private static final String VUFORIA_KEY =
            "AS+iypf/////AAABmVA+Shgi7EdsqH2d8iljwuh6/bNz0HoePOJW6LbOge+udoIv+21ZVTVE7Rbb6pcLmDZS0fP37WoY3OfgCQZBXOdLEpSXhJMHZuyD1V42qLPR9R+FarnuNWS21fr+EhRFPNatPM+riV2eQCS0WroErVmpvwDJUxQCI5Uk9ekS3TPW+9oEf/7V1OUr29wH6lMAwx0SOwTGW37/eaYWOpYpJRPJt9AtLrKupxQc6n9p87o/7oCBifNw0DLLb4L8WN3FaqhHuZ7iWYH3f1D7qcKetbfPW6oOEcTPhIhkERNOn+qiCj8zqJ626Bp1YLtOEEAdP+m25g7D3sJiwW32g/ekIFF8xNTNXwS830fKUHjB2z+D";


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
        robot.ParkSlideIn();

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


    public void StonePickup(){
        //Stone pickup
        robot.AutoArm.setPosition(robot.servoarmpickup);
        robot.AutoArmJoint.setPosition(robot.servojointdown);
        sleep(350);
        robot.Grab1.setPosition(robot.grab1close);
        robot.Grab2.setPosition(robot.grab2close);
        sleep(300);
        robot.AutoArmJoint.setPosition(robot.servojointup);
        robot.AutoArm.setPosition(servoarmbridge);
        sleep(200);

    }
    public void FirstStonePickup(){
        //First SkyStone pickup
        robot.AutoArm.setPosition(robot.servoarmpickup);
        robot.Grab1.setPosition(robot.grab1open);
        robot.Grab2.setPosition(robot.grab2open);
        robot.AutoArmJoint.setPosition(robot.servojointdown);
        sleep(350);
        robot.Grab1.setPosition(robot.grab1close);
        robot.Grab2.setPosition(robot.grab2close);
        sleep(300);
        robot.AutoArmJoint.setPosition(robot.servojointup);
        robot.AutoArm.setPosition(servoarmbridge);
        sleep(200);
    }
    public void StoneDeliver(){
        //Deliver
        robot.AutoArm.setPosition(robot.servoarmdeliver);
        robot.AutoArmJoint.setPosition(robot.servojointdeliver);
        sleep(150);
        robot.Grab1.setPosition(robot.grab1open);
        robot.Grab2.setPosition(robot.grab2open);
    }

    public void StoneBackDeliver(){
        //Deliver
        robot.AutoArm.setPosition(robot.servoarmdeliver);
        robot.AutoArmJoint.setPosition(robot.servojointdeliver);
        robot.Grab1.setPosition(robot.grab1open);
        robot.Grab2.setPosition(robot.grab2open);
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


    protected void initDogeCV() {
        // Start camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();
        this.phonedetector = new ImprovedSkystonePhoneDetector();
        this.phonedetector.useDefaults();
        webcam.setPipeline(phonedetector);
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

    protected void closeDogeCV(){
       webcam.stopStreaming();
       webcam.closeCameraDevice();
    }

    protected void closestreamDogeCV(){
        webcam.stopStreaming();
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
        if (phonedetector.getScreenPosition().x < 35) {
            value = 3;
        } else if (phonedetector.getScreenPosition().x < 90) {
            value = 1;
        } else if (phonedetector.getScreenPosition().x < 200) {
            value = 2;
        } else {
            value = 1;
        }
        telemetry.addData("Stone Position X", phonedetector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", phonedetector.getScreenPosition().y);
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
        telemetry.addData("Value", value);
        telemetry.update();

        return value;
    }


    public Vector2d BlueStoneMove(int skystone) {
        Vector2d position = new Vector2d(-60,38);
        if (skystone == 1 || skystone == 0) {
            position = new Vector2d(-60,38);
        } else if (skystone == 2) {
            position = new Vector2d(-52,38);
        } else if (skystone == 3) {
           position = new Vector2d(-44,38);
        }
return position;
    }

    public Vector2d RedStoneMove(int skystone) {
        Vector2d position = new Vector2d(-52,-35.5);
        if (skystone == 1 || skystone == 0) {
            position = new Vector2d(-52,-35.5);
        } else if (skystone == 2) {
            position = new Vector2d(-43,-35.5);
        } else if (skystone == 3) {
            position = new Vector2d(-39,-35.5);
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
        robot.Grab1.setPosition(robot.grab1home);
        robot.Grab2.setPosition(robot.grab2home);
        robot.AutoArm.setPosition(robot.servoarmhome);
        robot.AutoArm.setPwmDisable();
        robot.AutoArmJoint.setPwmDisable();
    }
}





