package org.firstinspires.ftc.teamcode.drive.SketchyTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.NotRoadRunner.SSAutoClasses;
import org.firstinspires.ftc.teamcode.drive.localizer.TwoTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class HouseNoArmBlueRoadRunnerAuto extends SSAutoClasses {

    int skystone = 0;

 Pose2d Start = new Pose2d(-36,60,0);
 Pose2d FirstBlock = new Pose2d(-60,37,0);
 Pose2d SecondBlock = new Pose2d(-52,37,0);
 Pose2d ThirdBlock = new Pose2d(-44,37,0);
     Pose2d FourthBlock = new Pose2d(-36,37,0);
     Pose2d FifthBlock = new Pose2d(-28,37,0);
     Pose2d SixthBlock = new Pose2d(-20,37,0);
     Pose2d Foundation = new Pose2d(28,35.5,0);
     Pose2d FoundationGrab = new Pose2d(60,32, Math.toRadians(270));
    Pose2d FoundationGrabForward = new Pose2d(60,27.5, Math.toRadians(270));
    Pose2d FoundationIn = new Pose2d(44,60,0);
Pose2d Park = new Pose2d(10,40,Math.toRadians(180));


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setLocalizer(new TwoTrackingWheelLocalizer(hardwareMap));
        initSensors();
        initWebcamDogeCV();
        drive.AutoArm.setPwmDisable();
        drive.setPoseEstimate(Start);

        waitForStart();
        skystone = runDetection(0);
        if (isStopRequested()) return;
        //drive.AutoArmRotate.setPosition(servorotaterblue);
        Trajectory theskystone = drive.trajectoryBuilder()
                .setReversed(true)
                // .strafeTo(StoneMove(skystone))
                .strafeTo(BlueStoneMove(skystone))
                /* .addMarker(()->{
                     new Vector2d(-50,50);
                 ((SampleMecanumDriveREVOptimized) drive).AutoArm.setPosition(servoarmdown);
                     return Unit.INSTANCE;})
 */
                //.strafeTo(new Vector2d(-60,37))
                .build();
        drive.followTrajectorySync(theskystone);
        drive.LEDDeliverMove(Foundation);
        drive.Yeet(FourthBlock,0,true);
        drive.Yeet(Foundation,0,false);
       /* drive.ReverseYeet(ThirdBlock);
        drive.Yeet(Foundation);
       /* drive.ReverseYeet(FifthBlock);
        drive.Yeet(Foundation); */
        drive.Yeet(SixthBlock,0,true);
        drive.Yeet(Foundation,0,false);








    }

}
