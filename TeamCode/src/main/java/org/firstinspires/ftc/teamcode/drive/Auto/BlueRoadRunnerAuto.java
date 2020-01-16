package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.TwoTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class BlueRoadRunnerAuto extends SSAutoClasses {

    int skystone = 0;

 Pose2d Start = new Pose2d(-36,60,0);
 Pose2d FirstBlock = new Pose2d(-60,37,0);
 Pose2d SecondBlock = new Pose2d(-52,37,0);
 Pose2d ThirdBlock = new Pose2d(-44,37,0);
     Pose2d FourthBlock = new Pose2d(-36,37,0);
     Pose2d FifthBlock = new Pose2d(-29,37,0);
     Pose2d SixthBlock = new Pose2d(-20,37,0);
     Pose2d Foundation = new Pose2d(48,35.5,0);
    Pose2d Foundation3 = new Pose2d(52,35.5,0);
     Pose2d FoundationGrab = new Pose2d(60,31.5, Math.toRadians(270));
    Pose2d FoundationGrabForward = new Pose2d(60,37.5, Math.toRadians(270));
 Pose2d FoundationIn = new Pose2d(42,55,0);
Pose2d Park = new Pose2d(5,40,0);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setLocalizer(new TwoTrackingWheelLocalizer(hardwareMap));
        initSensors();
        initWebcamDogeCV();
        drive.setPoseEstimate(Start);

        waitForStart();
        skystone = runDetection(0);
        sleep(150);
        if (isStopRequested()) return;
        Trajectory theskystone = drive.trajectoryBuilder()
                .setReversed(true)
                .addMarker( .15,
                        ()->{
                            drive.AutoArmRotate.setPosition(servorotaterblue);
                            return Unit.INSTANCE;})
                .strafeTo(BlueStoneMove(skystone))
                .build();


        drive.followTrajectorySync(theskystone);
        StonePickup();
        drive.BlueDeliverMove(Foundation);
        StoneDeliver();

        if (skystone == 1 || skystone == 0) {
            drive.BluePickupMove(FourthBlock);
        } else if (skystone == 2) {
            drive.BluePickupMove(FifthBlock);
        } else if (skystone == 3) {
            drive.BluePickupsixMove(SixthBlock);
        }

       StonePickup();
        drive.BlueDeliverMove(Foundation);
        StoneDeliver();

        if (skystone == 1 || skystone == 0) {
            drive.BluePickupsixMove(SixthBlock);
        } else if (skystone == 2) {
            drive.BluePickupsixMove(SixthBlock);
        } else if (skystone == 3) {
            drive.BluePickupMove(FifthBlock);
        }
        StonePickup();
        drive.BlueDeliverMove(Foundation3);
        StoneDeliver();

        ArmUpReset();
       drive.Yeet(FoundationGrab,270,false);
       drive.AutoArmRotate.setPosition(servorotatehome);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(5.5)
                        .build());
       GrabFoundation();
       sleep(350);
        drive.Yeet(FoundationIn,0, true);
        ReleaseFoundation();
        ArmKill();
        sleep(350);
        drive.Yeet(Park, 0,true);







    }

}
