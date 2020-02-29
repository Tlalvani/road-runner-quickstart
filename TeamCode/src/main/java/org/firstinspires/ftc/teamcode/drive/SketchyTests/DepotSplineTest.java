package org.firstinspires.ftc.teamcode.drive.SketchyTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class DepotSplineTest extends SSAutoClasses {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        initSensors();
        drive.AutoArm.setPwmDisable();
        //drive.setLocalizer(new MecanumDrive.MecanumLocalizer(drive));
        drive.setPoseEstimate(new Pose2d(-36,-60,0));

        Trajectory trajectory = drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(-60,-40,0))
               /* .addMarker(()->{
                    new Vector2d(-50,50);
                drive.AutoArm.setPosition(servoarmdown);
                    return Unit.INSTANCE;})
*/
               // .strafeTo(new Vector2d(-60,40))

                .build();

        waitForStart();

        if (isStopRequested()) return;
        //drive.AutoArmRotate.setPosition(servorotaterblue);
        drive.followTrajectorySync(trajectory);

        //drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));


    }
}
