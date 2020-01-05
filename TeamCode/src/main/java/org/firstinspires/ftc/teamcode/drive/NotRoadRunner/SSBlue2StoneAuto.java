package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

import static java.lang.Math.abs;


@Autonomous(name="BadSSBlue2StoneAuto", group="SS")  // @Autonomous(...) is the other common choice
public class SSBlue2StoneAuto extends SSOldAutoClasses
{
    int skystone = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initSensors();
        initWebcamDogeCV();
       robot.DrivebaseWithEncoders();
        telemetry.addData("Value",runDetection(0));
        telemetry.update();


        waitForStart();
        runtime.reset();
        skystone = runDetection(0);
        sleep(150);
        DriveTargetPosition(0,-1600,-1600,0);
        Drive(.5,.5); //.4
        telemetry.addData("power", robot.RB.getPower());
        while (robot.LB.isBusy() & robot.RF.isBusy() & opModeIsActive()) {
            robot.AutoArmRotate.setPosition(robot.servorotaterblue);
            telemetry.update();
        }
        Drive(0,0);

        //Square Wall
        DriveTargetPosition(-75,-75,-75,-75);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
        }
        Drive(0,0);

//Move to skystone
      StoneMove(skystone,15,200,330);

        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
        }
        Drive(0,0);

        period.reset();
        PDimu(0,period);
        StonePickup();

//Move to Foundation
        StoneMove(skystone,3615,3415,3315);
     Drive(RampPower(3615, .55,.35),RampPower(3615, .55,.35));
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            Drive(RampPower(3615, .55,.35),RampPower(3615, .55,.35));
            if (robot.RB.getCurrentPosition() < (robot.RB.getTargetPosition()-100) & robot.RB.getCurrentPosition() > (robot.RB.getTargetPosition()-3000)) {
                robot.AutoArmRotate.setPosition(robot.servorotateback);
            }

            else {
                robot.AutoArmRotate.setPosition(robot.servorotaterblue);
            }
        }
     Drive(0,0);

       StoneDeliver();

     period.reset();
     PDimu(0,period);

     //Move back for 2nd Skystone
     robot.AutoArmRotate.setPosition(robot.servorotateback);
        StoneMove(skystone,-2975,-2750,-2500);
       Drive(RampPower(-2975, -.3,-.5),RampPower(-2975, -.3,-.5));
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            Drive(RampPower(-2975, -.3,-.55),RampPower(-2975, -.3,-.55));
               if(Math.abs(robot.RB.getCurrentPosition() - robot.RB.getTargetPosition())<100){

               }
               else{
                robot.AutoArmRotate.setPosition(robot.servorotateback);}
        }
        Drive(0,0);

        robot.AutoArm.setPosition(robot.servoarmup);
        robot.AutoArmRotate.setPosition(robot.servorotaterblue);
        robot.AutoArmJoint.setPosition(robot.servojointup);
        sleep(500);
        StonePickup();

        period.reset();
        PDimu(0,period);

        //Move to Foundation
        StoneMove(skystone,2975,2750,2575);
        Drive(RampPower(2975, .6,.3),RampPower(2975, .6,.3));
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            Drive(RampPower(2975, .6,.3),RampPower(2975, .6,.3));
            if (robot.RB.getCurrentPosition() < (robot.RB.getTargetPosition()-100) & robot.RB.getCurrentPosition() > (robot.RB.getTargetPosition()-2500)) {
                robot.AutoArmRotate.setPosition(robot.servorotateback);
            }

            else {
                robot.AutoArmRotate.setPosition(robot.servorotaterblue);
            }
        }
        Drive(0,0);

        StoneDeliver();

        robot.AutoArmJoint.setPosition(robot.servojointup);
        robot.AutoArm.setPosition(robot.servoarmup);
        robot.Grab1.setPosition(robot.grab1home);
        robot.Grab2.setPosition(robot.grab2home);


        DriveTargetPosition(-300,-300,-300,-300);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            robot.AutoArmRotate.setPosition(robot.servorotatehome);
          //  liftup();
        }
        Drive(0,0);

        DriveTargetPosition(-100,100,100,-100);
        Drive(.3,.3);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
           // liftup();
        }
        Drive(0,0);

neg90();
/*while(robot.LiftCurrentPosition()<380){
    robot.Lift(.5);
}
robot.Lift(0);*/
imu(-90);

        robot.AutoArm.setPosition(robot.servoarmhome);
        robot.AutoArm.setPwmDisable();
        robot.AutoArmJoint.setPwmDisable();

        Drive(.2,.2);
        sleep(1000);
        Drive(0,0);

        robot.LeftLatch.setPosition(robot.leftlatchclose);
        robot.RightLatch.setPosition(robot.rightlatchclose);

        sleep(1000);
        /*DriveTargetPosition(-1250,-1250,-1250,-1250);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
        }
        Drive(0,0);
        */
        DriveTargetPosition(-400,-400,-400,-400);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
        }
        Drive(0,0);

        LeftAngleTurnTargetPosition(-1400,-1400);
        Drive(.4,0);
        LeftDrivebaseBusy();
        Drive(0,0);

        robot.LeftLatch.setPosition(robot.leftlatchopen);
        robot.RightLatch.setPosition(robot.rightlatchopen);

        DriveTargetPosition(100,100,100,100);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
       //     liftup();
        }
        Drive(0,0);

        DriveTargetPosition(800,-800,-800,800);
        Drive(.6,.6);
        DrivebaseBusy();
        Drive(0,0);

        DriveTargetPosition(-700,-700,-700,-700);
        Drive(.4,.4);
        DrivebaseBusy();
        Drive(0,0);

        while(opModeIsActive()) {
            sleep(1000000);
        }

    }
}

