package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;

/*
 * Sample tracking wheel localizer implementation assuming this two wheel configuration:
 *
 *    /--------------\
 *    |         ____ |
 *    | ||      ---- |
 *    | ||           |
 *    |              |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this is optimized significantly with REV bulk reads
 */
@Config
public class TwoTrackingWheelLocalizer extends com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.2; // in
    public static double strafeWHEEL_RADIUS = 1.14; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 3.5;  //5.1 // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 2.625; // 3.19 in; offset of the lateral wheel

    public static double FRONT_OFFSET = 3;
    public static double HORIZONTAL_OFFSET = -5.1;

    private ExpansionHubMotor leftEncoder, rightEncoder, frontEncoder;
    private ExpansionHubEx hub2;
    private List<ExpansionHubMotor> encoders;
    private BNO055IMU imu;

    public TwoTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(FRONT_OFFSET, LATERAL_DISTANCE, 0), // left
                new Pose2d(FORWARD_OFFSET, HORIZONTAL_OFFSET, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.get(ExpansionHubMotor.class,"leftintake");
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class,"Lift2");
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class,"rightintake");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
      /*  BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);*/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";

        imu.initialize(parameters);
        encoders = Arrays.asList(leftEncoder,rightEncoder,frontEncoder);

        for (ExpansionHubMotor encoder : encoders) {
                encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double strafeencoderTicksToInches(int ticks) {
        return strafeWHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
      /*  return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        ); */
        RevBulkData bulkData = hub2.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
       /* for (ExpansionHubMotor encoder : encoders) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(encoder)));
        }
      */    wheelPositions.add(-strafeencoderTicksToInches(bulkData.getMotorCurrentPosition(leftEncoder)));
        wheelPositions.add(-strafeencoderTicksToInches(bulkData.getMotorCurrentPosition(frontEncoder)));
        return wheelPositions;
    }
    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
