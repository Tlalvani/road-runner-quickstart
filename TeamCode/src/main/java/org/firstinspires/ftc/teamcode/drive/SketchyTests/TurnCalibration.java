package org.firstinspires.ftc.teamcode.drive.SketchyTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class TurnCalibration extends SSAutoClasses {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        initSensors();
        drive.AutoArm.setPwmDisable();
        drive.setLocalizer(new TwoTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0,0,0));



        waitForStart();

        if (isStopRequested()) return;
        //drive.AutoArmRotate.setPosition(servorotaterblue);
       drive.turnSync(7200);


        //drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));


    }
}
