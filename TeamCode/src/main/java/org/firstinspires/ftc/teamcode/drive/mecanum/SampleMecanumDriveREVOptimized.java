package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.grab1open;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.grab2open;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servoarmdeliver;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servoarmdown;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servoarmhigh;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servoarmup;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servojointdeliver;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servojointup;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servorotateback;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servorotatehome;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servorotateinvertblue;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servorotateinvertred;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servorotaterblue;
import static org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses.servorotatered;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.TwoTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import kotlin.Unit;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {


    private ExpansionHubEx hub;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    public List<ExpansionHubMotor> motors;
    private BNO055IMU imu;
    public DcMotor Lift1, Lift2, LeftIntake, RightIntake;
    public Servo LeftLatch, RightLatch, LeftClaw, RightClaw, LeftIntakeArm, RightIntakeArm,AutoArmRotate, Grab1, Grab2,ParkSlide;
    public ServoImplEx AutoArm,AutoArmJoint;
    double redyoffset = -36.5;

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();
        //StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        TwoTrackingWheelLocalizer localizer = new TwoTrackingWheelLocalizer(hardwareMap);
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);


        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "LF");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "LB");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "RB");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "RF");
        Lift1 = hardwareMap.dcMotor.get("Lift1");
        Lift2 = hardwareMap.dcMotor.get("Lift2");
        LeftIntake = hardwareMap.dcMotor.get("leftintake");
        RightIntake = hardwareMap.dcMotor.get("rightintake");

        LeftLatch = hardwareMap.servo.get("leftlatch");
        RightLatch = hardwareMap.servo.get("rightlatch");

        LeftClaw = hardwareMap.servo.get("leftclaw");
        RightClaw = hardwareMap.servo.get("rightclaw");

        LeftIntakeArm = hardwareMap.servo.get("leftintakearm");
        RightIntakeArm = hardwareMap.servo.get("rightintakearm");

        AutoArm =  hardwareMap.get(ServoImplEx.class, "autoarm");
        AutoArmJoint =  hardwareMap.get(ServoImplEx.class, "autoarmjoint");
        AutoArmRotate = hardwareMap.servo.get("autoarmrotate");
        Grab1 = hardwareMap.servo.get("grab1");
        Grab2 = hardwareMap.servo.get("grab2");
        ParkSlide = hardwareMap.servo.get("parkslide");
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
       setLocalizer(localizer);

    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {

        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(motor)));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }


    public void ConstantYeet(Pose2d position, double angle, boolean reversed){
        followTrajectorySync(
                trajectoryBuilder()
                        .setReversed(reversed)
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(angle)))
                        .build()
        );

    }

    public void BlackMagicYeet(Pose2d position, double angle, boolean reversed){
        followTrajectorySync(
                trajectoryBuilder()
                        .setReversed(reversed)
                        .splineTo(position,new SplineInterpolator(Math.toRadians(angle),0))
                        .build()
        );

    }
    public void Yeet(Pose2d position, double angle, boolean reversed){
        followTrajectorySync(
                trajectoryBuilder()
                        .setReversed(reversed)
                        .splineTo(position)
                        .build()
        );

    }

    public void BluePickupMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(()->{
                            AutoArmRotate.setPosition(servorotateback);
                            return Unit.INSTANCE;})
                        .setReversed(true)
                        .addMarker(new Vector2d(-20,37),
                                ()->{
                            AutoArmRotate.setPosition(servorotaterblue);
                            return Unit.INSTANCE;})
                        .splineTo(position)
                        .build()
        );

    }

    public void BluePickupsixMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(()->{
                            AutoArmRotate.setPosition(servorotateback);
                            return Unit.INSTANCE;})
                        .setReversed(true)
                        .addMarker(new Vector2d(-11,37),
                                ()->{
                            AutoArmRotate.setPosition(servorotaterblue);
                            return Unit.INSTANCE;})
                        .splineTo(position)
                        .build()
        );

    }
    public void BlueDeliverMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(
                                new Vector2d(-22,37),
                                ()->{
                                    AutoArmRotate.setPosition(servorotateback);
                            return Unit.INSTANCE;})
                        .addMarker(new Vector2d(20,37),
                                ()->{
                             AutoArmRotate.setPosition(servorotaterblue);
                            return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(0)))
                        .build()
        );

    }
    public void BlueLastDeliverMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(
                                new Vector2d(-22,37),
                                ()->{
                                    AutoArmRotate.setPosition(servorotateback);
                                    return Unit.INSTANCE;})
                        .addMarker(new Vector2d(20,37),
                                ()->{
                                    AutoArm.setPosition(servoarmup);
                                    AutoArmRotate.setPosition(servorotatehome);
                                    return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(0)))
                        .build()
        );

    }

    public void BlueFirstDeliverMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(
                                new Vector2d(-22,-redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotateback);
                                    return Unit.INSTANCE;})
                        .addMarker(new Vector2d(20,-redyoffset),
                                ()->{
                                   // AutoArm.setPosition(servoarmhigh);
                                    AutoArmRotate.setPosition(servorotatered);
                                    AutoArm.setPosition(servoarmup);
                                    AutoArmJoint.setPosition(servojointup);
                                    return Unit.INSTANCE;})
                        .addMarker(new Vector2d(40,-redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotatehome);
                                    return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(0)))
                        .build()
        );

    }

    public void RedFirstDeliverMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(
                                new Vector2d(-22,redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotateback);
                                    return Unit.INSTANCE;})
                        .addMarker(new Vector2d(20,redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotatered);
                                    AutoArm.setPosition(servoarmup);
                                    AutoArmJoint.setPosition(servojointup);
                                    return Unit.INSTANCE;})
                        .addMarker(new Vector2d(40,redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotatehome);
                                    return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(0)))
                        .build()
        );

    }
    public void RedPickupMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(()->{
                            AutoArmRotate.setPosition(servorotateback);
                            return Unit.INSTANCE;})
                        .setReversed(true)
                        .addMarker(new Vector2d(-12,redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotatered);
                                    return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(0)))
                        .build()
        );

    }

    public void RedReversePickupMove(Pose2d position){
        AutoArmRotate.setPosition(servorotateback);
        followTrajectorySync(
                trajectoryBuilder()
                        .setReversed(false)
                        .addMarker(new Vector2d(-21,redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotateinvertred);
                                    return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(180)))
                        .build()
        );

    }
    public void BlueReversePickupMove(Pose2d position){
        AutoArmRotate.setPosition(servorotateback);
        followTrajectorySync(
                trajectoryBuilder()
                        .setReversed(false)
                        .addMarker(new Vector2d(-21,37),
                                ()->{
                                    AutoArmRotate.setPosition(servorotateinvertblue);
                                    return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(180)))
                        .build()
        );

    }

    public void BlueReverseDeliverMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(180)))
                        .build()
        );

    }
    public void RedPickupsixMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(()->{
                            AutoArmRotate.setPosition(servorotateback);
                            return Unit.INSTANCE;})
                        .setReversed(true)
                        .addMarker(new Vector2d(-7,redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotatered);
                                    return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(0)))
                        .build()
        );

    }
    public void RedDeliverMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(
                                new Vector2d(-22,redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotateback);
                                    return Unit.INSTANCE;})
                        .addMarker(new Vector2d(30,redyoffset),
                                ()->{
                                    AutoArmRotate.setPosition(servorotatered);
                                    return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(0)))
                        .build()
        );

    }

    public void LEDDeliverMove(Pose2d position){
        followTrajectorySync(
                trajectoryBuilder()
                        .addMarker(
                                new Vector2d(-19,37),
                                ()->{ //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                            return Unit.INSTANCE;})
                        .addMarker(new Vector2d(40,37),
                        ()->{
                           // blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                            return Unit.INSTANCE;})
                        .splineTo(position,new ConstantInterpolator(Math.toRadians(0)))
                        .build()
        );

    }
}
