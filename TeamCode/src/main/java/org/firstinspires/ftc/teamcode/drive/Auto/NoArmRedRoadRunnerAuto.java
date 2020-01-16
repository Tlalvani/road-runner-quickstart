package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
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
public class NoArmRedRoadRunnerAuto extends SSAutoClasses {

    int skystone = 0;

    Pose2d Start = new Pose2d(-36,-60,0);
    Pose2d FirstBlock = new Pose2d(-53,-36.5,0);
    Pose2d SecondBlock = new Pose2d(-48,-36.5,0);
    Pose2d ThirdBlock = new Pose2d(-40,-36.5,0);
    Pose2d FourthBlock = new Pose2d(-32,-36.5,0);
    Pose2d FifthBlock = new Pose2d(-24,-36.5,0);
    Pose2d SixthBlock = new Pose2d(-16,-36.5,0);
    Pose2d Foundation = new Pose2d(48,-34.5,0);
    Pose2d Foundation3 = new Pose2d(52,-34.5,0);
    Pose2d FoundationGrab = new Pose2d(60,-32, Math.toRadians(90));
    Pose2d FoundationGrabForward = new Pose2d(60,-37.5, Math.toRadians(90));
    Pose2d FoundationIn = new Pose2d(42,-55,0);
    Pose2d Park = new Pose2d(5,-40,0);


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setLocalizer(new TwoTrackingWheelLocalizer(hardwareMap));
        initSensors();
        initDogeCV();
        drive.AutoArm.setPwmDisable();
        drive.setPoseEstimate(Start);

        waitForStart();
        skystone = runPhoneDetection(0);
        sleep(150);
        if (isStopRequested()) return;
        Trajectory theskystone = drive.trajectoryBuilder()
                .addMarker(
                        ()->{
                            //drive.AutoArmRotate.setPosition(servorotatered);
                            return Unit.INSTANCE;})
                .strafeTo(RedStoneMove(skystone))

                .build();


      drive.followTrajectorySync(theskystone);


        drive.Yeet(Foundation,0,false);
        drive.Yeet(FourthBlock,0,true);
        drive.Yeet(Foundation,0,false);
       /* drive.ReverseYeet(ThirdBlock);
        drive.Yeet(Foundation);
       /* drive.ReverseYeet(FifthBlock);
        drive.Yeet(Foundation); */
        drive.Yeet(SixthBlock,0,true);
        drive.Yeet(Foundation,0,false);

       drive.Yeet(FoundationGrab,90,false);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(5.5)
                        .build());
       GrabFoundation();
       sleep(350);
        drive.Yeet(FoundationIn,0, true);
        ReleaseFoundation();

        drive.Yeet(Park, 0,true);
        drive.Yeet(FirstBlock,0,true);







    }

}
