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
public class SketchBlueRoadRunnerAuto extends SSAutoClasses {

    int skystone = 0;

 Pose2d Start = new Pose2d(-36,60,0);
 Pose2d FirstBlock = new Pose2d(-60,36,0);
 Pose2d SecondBlock = new Pose2d(-52,36,0);
 Pose2d ThirdBlock = new Pose2d(-40,36,0);
     Pose2d FourthBlock = new Pose2d(-41,37,0);
     Pose2d FifthBlock = new Pose2d(-34,37,0);
     Pose2d SixthBlock = new Pose2d(-24,37,0);
     Pose2d Foundation = new Pose2d(48,36.5,0);
    Pose2d Foundation3 = new Pose2d(52,36.5,0);
     Pose2d FoundationGrab = new Pose2d(60,30.5, Math.toRadians(270));
    Pose2d FoundationGrabForward = new Pose2d(60,25.5, Math.toRadians(270));
 Pose2d FoundationIn = new Pose2d(42,55,0);
 Pose2d ParkY = new Pose2d(42,40,0);
Pose2d Park = new Pose2d(5,36,0);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setLocalizer(new TwoTrackingWheelLocalizer(hardwareMap));
        initSensors();
        initWebcamDogeCV();
        drive.setPoseEstimate(Start);

        waitForStart();
        skystone = runDetection(0);
        sleep(50);
        if (isStopRequested()) return;
        Trajectory theskystone = drive.trajectoryBuilder()
                .setReversed(true)
                .addMarker( .15,
                        ()->{
                            drive.AutoArmRotate.setPosition(servorotaterblue);
                            drive.AutoArmJoint.setPosition(servojointup);
                            return Unit.INSTANCE;})
                .strafeTo(BlueStoneMove(skystone))
                .build();


        drive.followTrajectorySync(theskystone);
        FirstStonePickup();
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

      //drive.setPoseEstimate(new Pose2d(drive.getLocalizer().getPoseEstimate().getX(),drive.getLocalizer().getPoseEstimate().getY()+1,drive.getLocalizer().getPoseEstimate().getHeading()));

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

        if (skystone == 1 || skystone == 0) {
            drive.BluePickupMove(FifthBlock);
        } else if (skystone == 2) {
            drive.BluePickupMove(FourthBlock);
        } else if (skystone == 3) {
            drive.BluePickupMove(FourthBlock);

        }
        StonePickup();
        drive.AutoArmRotate.setPosition(servorotateback);
        drive.BlueDeliverMove(Foundation3);
        StoneDeliver();

        sleep(100);
        ArmUpReset();
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(FoundationGrab,new ConstantInterpolator(Math.toRadians(270)))
                        .addMarker(()->{drive.AutoArmRotate.setPosition(servorotatehome);
                        return Unit.INSTANCE;})
                        .splineTo(FoundationGrabForward,new ConstantInterpolator(Math.toRadians(270)))
                        .build());
       GrabFoundation();
       sleep(150);
       drive.Yeet(FoundationIn,0,true);
        ReleaseFoundation();
        ArmKill();
        sleep(50);
        drive.ConstantYeet(Park, 0,true);







    }

}
