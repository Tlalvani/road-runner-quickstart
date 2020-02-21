package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone.TFOD_MODEL_ASSET;

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
abstract public class SSOldAutoClasses extends LinearOpMode {
    //150
    //300
    //550
    //750
    //1040
    //1300

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
PIDFCoefficients OldPID = new PIDFCoefficients(10,3,0,0);
    private static final String VUFORIA_KEY =
            "AS+iypf/////AAABmVA+Shgi7EdsqH2d8iljwuh6/bNz0HoePOJW6LbOge+udoIv+21ZVTVE7Rbb6pcLmDZS0fP37WoY3OfgCQZBXOdLEpSXhJMHZuyD1V42qLPR9R+FarnuNWS21fr+EhRFPNatPM+riV2eQCS0WroErVmpvwDJUxQCI5Uk9ekS3TPW+9oEf/7V1OUr29wH6lMAwx0SOwTGW37/eaYWOpYpJRPJt9AtLrKupxQc6n9p87o/7oCBifNw0DLLb4L8WN3FaqhHuZ7iWYH3f1D7qcKetbfPW6oOEcTPhIhkERNOn+qiCj8zqJ626Bp1YLtOEEAdP+m25g7D3sJiwW32g/ekIFF8xNTNXwS830fKUHjB2z+D";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /* Constructor */
    public SSOldAutoClasses() {

    }

    //FUNCTIONS

    public void initSensors() {
        robot.init(hardwareMap);
        robot.RB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,OldPID);
        robot.RF.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,OldPID);
        robot.LB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,OldPID);
        robot.LF.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,OldPID);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters imuparameters = new BNO055IMU.Parameters();
        imuparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuparameters.loggingEnabled = true;
        imuparameters.loggingTag = "IMU";
        imuparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuparameters);

        // Set up our telemetry dashboard
        composeTelemetry();

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
        sleep(500);
        robot.Grab1.setPosition(robot.grab1close);
        robot.Grab2.setPosition(robot.grab2close);
        sleep(400);
        robot.AutoArmJoint.setPosition(robot.servojointup);
        robot.AutoArm.setPosition(robot.servoarmdown);
        sleep(400);
    }

    public void StoneDeliver(){
        //Deliver
        robot.AutoArm.setPosition(robot.servoarmdeliver);
        sleep(250);
        robot.AutoArmJoint.setPosition(robot.servojointdeliver);
        sleep(250);
        robot.Grab1.setPosition(robot.grab1open);
        robot.Grab2.setPosition(robot.grab2open);
        sleep(250);

    }
    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    protected void initWebcamVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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
    protected int runoldDetection(int value) {
        Recognition skystone = null;
        boolean stone = false;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List < Recognition > updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition: updatedRecognitions) {
                    if (recognition.getLabel() == "Skystone") {
                        skystone = recognition;
                        stone = true;
                    }
                }
                if (updatedRecognitions.size() > 0 && stone) {

                    if (skystone.getLeft() < 150) {
                        value = 3;
                    } else if (skystone.getLeft() < 400) {
                        value = 2;
                    } else if (skystone.getLeft() > 400) {
                        value = 1;
                    } else {
                        value = 1;
                    }
                    telemetry.addData("Left Position", skystone.getLeft());
                    telemetry.addData("Stone", skystone.getLabel());
                    telemetry.addData("Value", value);
                    telemetry.update();
                }

            }
        }

        return value;
    }

    protected int runDetection(int value) {
        if (detector.getScreenPosition().x < 60) {
            value = 2;
        } else if (detector.getScreenPosition().x < 130) {
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
    protected int runoldPhoneDetection(int value) {
        Recognition skystone = null;
        boolean stone = false;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List < Recognition > updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition: updatedRecognitions) {
                    if (recognition.getLabel() == "Skystone") {
                        skystone = recognition;
                        stone = true;
                    }
                }
                if (updatedRecognitions.size() > 0 && stone) {

                    if (skystone.getLeft() > 20) {
                        value = 3;
                    } else if (skystone.getLeft() < 120) {
                        value = 2;
                    } else if (skystone.getRight() > 300) {
                        value = 1;
                    } else {
                        value = 1;
                    }
                    telemetry.addData("Right Position", skystone.getRight());
                    telemetry.addData("Stone", skystone.getLabel());
                    telemetry.addData("Value", value);
                    telemetry.update();
                }

            }
        }

        return value;
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func < String > () {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func < String > () {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func < String > () {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func < String > () {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func < String > () {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func < String > () {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func < String > () {
                    @Override
                    public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel +
                                        gravity.yAccel * gravity.yAccel +
                                        gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void imu(double degrees) {
        robot.DrivebaseWithEncoders();
        telemetry.update();
        double leftSpeedt; //Power to feed the motors
        double rightSpeedt;
        double Z = angles.firstAngle;

        while (angles.firstAngle < (degrees - robot.AngleTolerance) || angles.firstAngle > (degrees + robot.AngleTolerance) & opModeIsActive()) {
            // position turns left, think of a x,y coordinate system
            telemetry.update();

            leftSpeedt = -((degrees - angles.firstAngle) / robot.divisorforimu); //Calculate speed for each side
            rightSpeedt = ((degrees - angles.firstAngle) / robot.divisorforimu); //See Gyro Straight video for detailed explanation

            telemetry.update();
            if (leftSpeedt > 0) {
                leftSpeedt = Range.clip(leftSpeedt, robot.minspeedimu, robot.maxspeedimu);
            } else if (leftSpeedt < 0) {
                leftSpeedt = Range.clip(leftSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu));
            }


            if (rightSpeedt > 0) {
                rightSpeedt = Range.clip(rightSpeedt, robot.minspeedimu, robot.maxspeedimu);
            } else if (rightSpeedt < 0) {
                rightSpeedt = Range.clip(rightSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu));
            }
            telemetry.update();
            telemetry.update();
            Drive(leftSpeedt, rightSpeedt);
            telemetry.update();

            idle();
        }
        Drive(0, 0);

    }

 public void neg90(){
        DriveTargetPosition(705,705,-705,-705);
        Drive(.5,-.5);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
           // liftup();
            Drive(RampPower(705, .5,.1),RampPower(-705, -.1,-.5));
            telemetry.update();
        }
        Drive(0,0);
    }
    public void norm90(){
        DriveTargetPosition(-705,-705,705,705);
        Drive(.5,-.5);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            //liftup();
            robot.AutoArmRotate.setPosition(robot.servorotatehome);
            Drive(RampPower(705, .5,.1),RampPower(-705, -.1,-.5));
            telemetry.update();
        }
        Drive(0,0);
    }
    public void oneeightyimu(double degrees) {
        telemetry.update();
        double leftSpeedt; //Power to feed the motors
        double rightSpeedt;

        double Z = angles.firstAngle;


        while (robot.currentangle < (180 - robot.AngleTolerance) || robot.currentangle > (180 + robot.AngleTolerance) & opModeIsActive()) {
            // positie turns left, think of a x,y coordinate system
            telemetry.update();

            if (angles.firstAngle < 0) {
                robot.currentangle = angles.firstAngle + 360;
            } else {
                robot.currentangle = angles.firstAngle;
            }

            leftSpeedt = -((180 - robot.currentangle) / robot.divisorforimu); //Calculate speed for each side
            rightSpeedt = ((180 - robot.currentangle) / robot.divisorforimu); //See Gyro Straight video for detailed explanation

            telemetry.update();
            if (leftSpeedt > 0) {
                leftSpeedt = Range.clip(leftSpeedt, robot.minspeedimu, robot.maxspeedimu);
            } else if (leftSpeedt < 0) {
                leftSpeedt = Range.clip(leftSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu));
            }


            if (rightSpeedt > 0) {
                rightSpeedt = Range.clip(rightSpeedt, robot.minspeedimu, robot.maxspeedimu);
            } else if (rightSpeedt < 0) {
                rightSpeedt = Range.clip(rightSpeedt, -(robot.maxspeedimu), -(robot.minspeedimu));
            }
            telemetry.update();
            Drive(leftSpeedt, rightSpeedt);
            telemetry.update();

            idle();
        }
        Drive(0, 0);

    }

    public void PDimu(double degrees, ElapsedTime looptime) {
        robot.DrivebaseWithEncoders();
        telemetry.update();
        double leftSpeedt; //Power to feed the motors
        double rightSpeedt;

        double Z = angles.firstAngle;

        double error;
        double prevError = 0;

        double derivative;
        double kD = .1;
        double AngleTolerance = robot.AngleTolerance;




        while (angles.firstAngle < (degrees - AngleTolerance) || angles.firstAngle > (degrees + AngleTolerance) & opModeIsActive() & !isStopRequested()) {
            // positie turns left, think of a x,y coordinate system
            telemetry.update();

            error = (degrees - angles.firstAngle); //get the current error

            dt = looptime.seconds(); //set the time since the last loop

            derivative = (error - prevError) / dt;
            looptime.reset(); //reset the loop timer

            leftSpeedt = -((error) / robot.divisorforimu); //Calculate speed for each side
            rightSpeedt = ((error) / robot.divisorforimu); //See Gyro Straight video for detailed explanation

            leftSpeedt = leftSpeedt + (derivative * kD);
            rightSpeedt = rightSpeedt + (derivative * kD);
            telemetry.update();

            if (leftSpeedt > 0) {
                leftSpeedt = Range.clip(leftSpeedt, 0, robot.pdmaxspeedimu);
            } else if (leftSpeedt < 0) {
                leftSpeedt = Range.clip(leftSpeedt, -(robot.pdmaxspeedimu), -(0));
            }


            if (rightSpeedt > 0) {
                rightSpeedt = Range.clip(rightSpeedt, 0, robot.pdmaxspeedimu);
            } else if (rightSpeedt < 0) {
                rightSpeedt = Range.clip(rightSpeedt, -(robot.pdmaxspeedimu), -(0));
            }
            telemetry.update();
            Drive(leftSpeedt, rightSpeedt);
            prevError = error; //set the previous error for the next loop cycle
            telemetry.update();

            idle();
        }
        Drive(0, 0);

    }
    public void Recorrectimu(double degrees, ElapsedTime looptime) {

        telemetry.update();
        double leftSpeedt; //Power to feed the motors
        double rightSpeedt;

        double Z = angles.firstAngle;

        double error;
        double prevError = 0;

        double derivative;
        double kD = .1;
        double AngleTolerance = 1.1;



        period.reset();
        if (angles.firstAngle < (degrees - AngleTolerance) || angles.firstAngle > (degrees + AngleTolerance) & opModeIsActive() & !isStopRequested()) {
            //robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            // positie turns left, think of a x,y coordinate system
            telemetry.update();

            error = (degrees - angles.firstAngle); //get the current error

            dt = looptime.seconds(); //set the time since the last loop

            derivative = (error - prevError) / dt;
            looptime.reset(); //reset the loop timer

            leftSpeedt = -((error) / robot.divisorforimu); //Calculate speed for each side
            rightSpeedt = ((error) / robot.divisorforimu); //See Gyro Straight video for detailed explanation

            leftSpeedt = leftSpeedt + (derivative * kD);
            rightSpeedt = rightSpeedt + (derivative * kD);
            telemetry.update();

            if (leftSpeedt > 0) {
                leftSpeedt = Range.clip(leftSpeedt, 0, robot.pdmaxspeedimu);
            } else if (leftSpeedt < 0) {
                leftSpeedt = Range.clip(leftSpeedt, -(robot.pdmaxspeedimu), -(0));
            }


            if (rightSpeedt > 0) {
                rightSpeedt = Range.clip(rightSpeedt, 0, robot.pdmaxspeedimu);
            } else if (rightSpeedt < 0) {
                rightSpeedt = Range.clip(rightSpeedt, -(robot.pdmaxspeedimu), -(0));
            }
            telemetry.update();
            Drive(leftSpeedt, rightSpeedt);
            prevError = error; //set the previous error for the next loop cycle
            telemetry.update();

            idle();
        }
        else {
            Drive(.1, .1);
           // robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }

    }
    public void StoneMove(int skystone, int pos1, int pos2, int pos3) {
        if (skystone == 1 || skystone == 0) {
            DriveTargetPosition(pos1, pos1, pos1, pos1);
        } else if (skystone == 2) {
            DriveTargetPosition(pos2, pos2, pos2, pos2);
        } else if (skystone == 3) {
            DriveTargetPosition(pos3, pos3, pos3, pos3);
        }

    }
}





