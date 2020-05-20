package org.firstinspires.ftc.teamcode.drive.NotRoadRunner.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
public class NewRedRoadRunnerAuto extends SSAutoClasses {

    int skystone = 0;
double yvalue = -38.5;

 Pose2d Start = new Pose2d(-36,-60,0);
 Pose2d FirstBlock = new Pose2d(-53,-37,0);
 Pose2d SecondBlock = new Pose2d(-48,-37,0);
 Pose2d ThirdBlock = new Pose2d(-40,yvalue,0);

 Pose2d FourthBlock = new Pose2d(-43,yvalue,Math.toRadians(180));
 Pose2d FifthBlock = new Pose2d(-34,yvalue,Math.toRadians(180));
 Pose2d SixthBlock = new Pose2d(-26,yvalue,Math.toRadians(180));

 Pose2d Foundation = new Pose2d(54,-34,0);

    Pose2d FoundationGrab = new Pose2d(56,-34, Math.toRadians(90));
    Pose2d FoundationGrabForward = new Pose2d(56,-28, Math.toRadians(90));

 Pose2d FoundationClose = new Pose2d(26,-42, Math.toRadians(0));
    Pose2d FoundationCloseStart = new Pose2d(22,yvalue,Math.toRadians(180));
 Pose2d FoundationCloseScore = new Pose2d(26,yvalue,Math.toRadians(180));

 Pose2d FoundationIn = new Pose2d(47,-34,Math.toRadians(180));

Pose2d Park = new Pose2d(9,-36,Math.toRadians(180));

    private DriveConstraints constraints = new DriveConstraints(40, 40.0, 0.0, Math.toRadians(135.0), Math.toRadians(135.0), 0.0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setLocalizer(new TwoTrackingWheelLocalizer(hardwareMap));
        initSensors();
        initDogeCV();
        drive.setPoseEstimate(Start);
        TrajectoryBuilder builder1 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints);
        waitForStart();
        skystone = runPhoneDetection(0);
        sleep(50);
        if (isStopRequested()) return;
       if(skystone==3) {
           Trajectory theskystone = drive.trajectoryBuilder()
                   .setReversed(true)
                   .addMarker(
                           () -> {
                               drive.AutoArmRotate.setPosition(servorotatered);
                               return Unit.INSTANCE;
                           })
                   .strafeTo(RedStoneMove(skystone))
                 //  .splineTo(SecondBlock)
                   .build();
           drive.followTrajectorySync(theskystone);
       }
       else if(skystone==2){
           Trajectory theskystone = drive.trajectoryBuilder()
                   .setReversed(true)
                   .addMarker(
                           () -> {
                               drive.AutoArmRotate.setPosition(servorotatered);
                               return Unit.INSTANCE;
                           })
                   //.strafeTo(RedStoneMove(skystone))
                   .splineTo(SecondBlock)
                   .build();
           drive.followTrajectorySync(theskystone);
       }
       else if(skystone==1){
           Trajectory theskystone = drive.trajectoryBuilder()
                   .setReversed(true)
                   .addMarker(
                           () -> {
                               drive.AutoArmRotate.setPosition(servorotatered);
                               return Unit.INSTANCE;
                           })
                   //.strafeTo(RedStoneMove(skystone))
                   .splineTo(FirstBlock)
                   .build();
           drive.followTrajectorySync(theskystone);
       }

        closeDogeCV();

        FirstStonePickup();
        drive.RedDeliverMove(Foundation);
        StoneDeliver();
       sleep(50);

        drive.AutoArmJoint.setPosition(servojointdeliver);
        drive.AutoArm.setPosition(servoarmhigh);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(FoundationGrab,new ConstantInterpolator(Math.toRadians(90)))
                        .splineTo(FoundationGrabForward,new ConstantInterpolator(Math.toRadians(90)))
                        //.forward(4)
                        .build());

        GrabFoundation();
        drive.Yeet(FoundationClose,0,true);
        ReleaseFoundation();
        sleep(50);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .addMarker(
                                ()->{
                                    drive.AutoArmRotate.setPosition(servorotateback);
                                    drive.AutoArm.setPosition(servoarmup);
                                    return Unit.INSTANCE;})
                        .setReversed(true)
                        .splineTo(FoundationCloseStart,new SplineInterpolator(Math.toRadians(-180.0),0.0))
                        .build());

        drive.AutoArmJoint.setPosition(servojointdeliver);
        drive.AutoArm.setPosition(servoarmdeliver);
        if (skystone == 1 || skystone == 0) {
            drive.RedReversePickupMove(FourthBlock);
        } else if (skystone == 2) {
            drive.RedReversePickupMove(FifthBlock);
        } else if (skystone == 3) {
            drive.RedReversePickupMove(SixthBlock);
            sleep(350);
        }


        StonePickup();
        drive.AutoArmRotate.setPosition(servorotateredpartial); //Score right

        if (skystone == 1 || skystone == 0) {
        } else if (skystone == 2) {
        } else if (skystone == 3) {
            sleep(150);
        }

        drive.Yeet(FoundationCloseScore,180,true);
        StoneBackDeliver();

        if (skystone == 1 || skystone == 0) {
            drive.RedReversePickupMove(SixthBlock);
            sleep(350);
        } else if (skystone == 2) {
            drive.RedReversePickupMove(SixthBlock);
            sleep(350);
        } else if (skystone == 3) {
            drive.RedReversePickupMove(FifthBlock);
            sleep(150);
        }

        StonePickup();
        drive.AutoArmRotate.setPosition(servorotateback);

        //Score Straight

        if (skystone == 1 || skystone == 0) {
            sleep(150);
        } else if (skystone == 2) {
            sleep(150);
        } else if (skystone == 3) {
        }
        drive.Yeet(FoundationCloseScore,180,true);
        StoneBackDeliver();

        if (skystone == 1 || skystone == 0) {
            drive.RedReversePickupMove(FifthBlock);
        } else if (skystone == 2) {
            drive.RedReversePickupMove(FourthBlock);
        } else if (skystone == 3) {
            drive.RedReversePickupMove(FourthBlock);
        }

        StonePickup();
        drive.AutoArmRotate.setPosition(servorotateredpartial);
        drive.Yeet(FoundationIn,180,true);
        StoneBackDeliver();
        sleep(200);
        ArmUpReset();
        drive.AutoArmRotate.setPosition(servorotatehome);
        sleep(300);
        ArmKill();
        drive.ConstantYeet(Park,180,false);



    }

}
