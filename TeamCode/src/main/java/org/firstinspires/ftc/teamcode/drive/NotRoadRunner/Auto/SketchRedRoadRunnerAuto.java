package org.firstinspires.ftc.teamcode.drive.NotRoadRunner.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses;
import org.firstinspires.ftc.teamcode.drive.localizer.TwoTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class SketchRedRoadRunnerAuto extends SSAutoClasses {

    int skystone = 0;


 Pose2d Start = new Pose2d(-36,-60,0);
 Pose2d FirstBlock = new Pose2d(-53,-35.5,0);
 Pose2d SecondBlock = new Pose2d(-48,-35.5,0);
 Pose2d ThirdBlock = new Pose2d(-40,-35.5,0);
 Pose2d FourthBlock = new Pose2d(-29,-35.5,0);
 Pose2d FifthBlock = new Pose2d(-22,-35.5,0);
 Pose2d SixthBlock = new Pose2d(-14,-35.5,0);
 Pose2d Foundation = new Pose2d(54,-33.5,0);
 Pose2d Foundation2 = new Pose2d(56,-33.5,0);
 Pose2d Foundation3 = new Pose2d(58,-33.5,0);
 Pose2d FoundationGrab = new Pose2d(62,-29, Math.toRadians(90));
 Pose2d FoundationGrabForward = new Pose2d(62,-26, Math.toRadians(90));
 Pose2d FoundationIn = new Pose2d(42,-55,0);
Pose2d Park = new Pose2d(3,-36,0);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setLocalizer(new TwoTrackingWheelLocalizer(hardwareMap));
        initSensors();
        initDogeCV();
        drive.setPoseEstimate(Start);

        waitForStart();
        skystone = runPhoneDetection(0);
        sleep(50);
        if (isStopRequested()) return;
        Trajectory theskystone = drive.trajectoryBuilder()
                .setReversed(true)
                .addMarker(
                        ()->{
                            drive.AutoArmRotate.setPosition(servorotatered);
                            return Unit.INSTANCE;})
                .strafeTo(RedStoneMove(skystone))
                .build();


        drive.followTrajectorySync(theskystone);
        FirstStonePickup();
        drive.RedDeliverMove(Foundation);
        StoneDeliver();

        if (skystone == 1 || skystone == 0) {
            drive.RedPickupMove(FourthBlock);
        } else if (skystone == 2) {
            drive.RedPickupMove(FifthBlock);
        } else if (skystone == 3) {
            drive.RedPickupsixMove(SixthBlock);
        }

       StonePickup();
        drive.RedDeliverMove(Foundation2);
        StoneDeliver();

        if (skystone == 1 || skystone == 0) {
            drive.RedPickupsixMove(SixthBlock);
        } else if (skystone == 2) {
            drive.RedPickupsixMove(SixthBlock);
        } else if (skystone == 3) {
            drive.RedPickupMove(FifthBlock);
        }

        StonePickup();
        drive.RedDeliverMove(Foundation3);
        StoneDeliver();

        if (skystone == 1 || skystone == 0) {
            drive.RedPickupMove(FifthBlock);
        } else if (skystone == 2) {
            drive.RedPickupMove(FourthBlock);
        } else if (skystone == 3) {
            drive.RedPickupMove(FourthBlock);

        }

        StonePickup();
        drive.RedDeliverMove(Foundation3);
        StoneDeliver();
        sleep(100);
        ArmUpReset();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(FoundationGrab,new ConstantInterpolator(Math.toRadians(90)))
                        .addMarker(()->{drive.AutoArmRotate.setPosition(servorotatehome);
                            return Unit.INSTANCE;})
                        .splineTo(FoundationGrabForward,new ConstantInterpolator(Math.toRadians(90)))
                        .build());
       GrabFoundation();
       sleep(150);
        drive.Yeet(FoundationIn,0, true);
        ReleaseFoundation();
        ArmKill();
        sleep(50);
        drive.ConstantYeet(Park,0, true);







    }

}
