package org.firstinspires.ftc.teamcode.drive.NotRoadRunner.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class NewBlueRoadRunnerAuto extends SSAutoClasses {

    int skystone = 0;
double yvalue = 38.5;

    private DriveConstraints constraints = new DriveConstraints(60, 70.0, 0.0, Math.toRadians(135.0), Math.toRadians(135.0), 0.0);

 Pose2d Start = new Pose2d(-36,60,0);
 Pose2d FirstBlock = new Pose2d(-53,yvalue,0);
 Pose2d SecondBlock = new Pose2d(-48,yvalue,0);
 Pose2d ThirdBlock = new Pose2d(-40,yvalue,0);

 Pose2d FourthBlock = new Pose2d(-51,yvalue,Math.toRadians(180));
 Pose2d FifthBlock = new Pose2d(-42,yvalue,Math.toRadians(180));
 Pose2d SixthBlock = new Pose2d(-34,yvalue,Math.toRadians(180));

 Pose2d Foundation = new Pose2d(50,35,0);

    Pose2d FoundationGrab = new Pose2d(50,30.5, Math.toRadians(270));
    Pose2d FoundationGrabForward = new Pose2d(50,25.5, Math.toRadians(270));


 Pose2d FoundationClose = new Pose2d(16,40, Math.toRadians(0));
    Pose2d FoundationCloseStart = new Pose2d(14,yvalue+2,Math.toRadians(180));
 Pose2d FoundationCloseScore = new Pose2d(22,yvalue+1,Math.toRadians(180));

 Pose2d FoundationIn = new Pose2d(45,40,Math.toRadians(180));
 Vector2d Foundationin = new Vector2d(45,39);

Pose2d Park = new Pose2d(6,42,Math.toRadians(180));


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setLocalizer(new TwoTrackingWheelLocalizer(hardwareMap));
        initSensors();
        initWebcamDogeCV();
        drive.setPoseEstimate(Start);

        waitForStart();
        skystone = runDetection(0);
        sleep(25);
        if (isStopRequested()) return;
        Trajectory theskystone = drive.trajectoryBuilder()
                .setReversed(true)
                .addMarker( .05,
                        ()->{
                            drive.AutoArmRotate.setPosition(servorotaterblue);
                            drive.Grab2.setPosition(grab2close);
                           // drive.AutoArmJoint.setPosition(.5);
                            return Unit.INSTANCE;})
                .strafeTo(BlueStoneMove(skystone))
                .build();

        drive.followTrajectorySync(theskystone);
      // closeDogeCV();
        FirstStonePickup();
        drive.RedFirstDeliverMove(Foundation);
        drive.AutoArmJoint.setPosition(servojointdeliver);

       /* drive.BlueDeliverMove(Foundation);
        StoneDeliver();
       sleep(50);

        drive.AutoArmJoint.setPosition(servojointdeliver);
        drive.AutoArm.setPosition(servoarmhigh);
*/
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(FoundationGrab,new ConstantInterpolator(Math.toRadians(270)))
                        .addMarker(()->{drive.Grab1.setPosition(grab1open);
                            drive.Grab2.setPosition(grab2open);
                            return Unit.INSTANCE;})
                        .splineTo(FoundationGrabForward,new ConstantInterpolator(Math.toRadians(270)))
                        .build());

        GrabFoundation();
        drive.AutoArmJoint.setPosition(servojointup);

        drive.Yeet(FoundationClose,0,true);
        ReleaseFoundation();
        sleep(50);

   /*     drive.AutoArmRotate.setPosition(servorotateback);
        drive.AutoArm.setPosition(servoarmup);
        drive.turnSync(180); */


        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(false)
                        .addMarker(.1,
                                ()->{
                                    drive.AutoArmRotate.setPosition(servorotateback);
                                    drive.AutoArm.setPosition(servoarmup);
                                    return Unit.INSTANCE;})
                        .setReversed(false)
                        .splineTo(FoundationCloseStart,new SplineInterpolator(0.0,Math.toRadians(180.0)))
                        .build());

        drive.AutoArmJoint.setPosition(servojointdeliver);
        drive.AutoArm.setPosition(servoarmdeliver);

        if (skystone == 1 || skystone == 0) {
            drive.BlueReversePickupMove(FourthBlock);
        } else if (skystone == 2) {
            drive.BlueReversePickupMove(FifthBlock);
        } else if (skystone == 3) {
            drive.BlueReversePickupMove(SixthBlock);
            sleep(350);
        }

        StonePickup();
        drive.AutoArmRotate.setPosition(servorotatebluepartial); //Score right
       // drive.AutoArmRotate.setPosition(servorotateback);

        if (skystone == 1 || skystone == 0) {
        } else if (skystone == 2) {
        } else if (skystone == 3) {
            sleep(150);
        }

        drive.BlueReverseDeliverMove(FoundationCloseScore);

       StoneBackDeliver();
       drive.setPoseEstimate(new Pose2d(drive.getLocalizer().getPoseEstimate().getX(),drive.getLocalizer().getPoseEstimate().getY()-1.5,drive.getLocalizer().getPoseEstimate().getHeading()));



        if (skystone == 1 || skystone == 0) {
            drive.BlueReversePickupMove(SixthBlock);
            sleep(350);
        } else if (skystone == 2) {
            drive.BlueReversePickupMove(SixthBlock);
            sleep(350);
        } else if (skystone == 3) {
            drive.BlueReversePickupMove(FifthBlock);
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
        drive.BlueReverseDeliverMove(FoundationCloseScore);
        StoneBackDeliver();

        if (skystone == 1 || skystone == 0) {
            drive.BlueReversePickupMove(FifthBlock);
        } else if (skystone == 2) {
            drive.BlueReversePickupMove(FourthBlock);
        } else if (skystone == 3) {
            drive.BlueReversePickupMove(FourthBlock);
        }

        StonePickup();
        drive.AutoArmRotate.setPosition(servorotatebluepartial);

        drive.ConstantYeet(FoundationIn,180,true);
        StoneBackDeliver();
        sleep(100);
        ArmUpReset();
        drive.AutoArmRotate.setPosition(servorotatehome);
        sleep(300);
        ArmKill();

        drive.LeftClaw.setPosition(.5);
        //drive.ConstantYeet(Park,180,false);

        TrajectoryBuilder builder1 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints);

        Trajectory park = builder1
                .setReversed(false)
                .splineTo(Park,new ConstantInterpolator(Math.toRadians(180)))
                .build();

        drive.followTrajectorySync(park);





    }

}
