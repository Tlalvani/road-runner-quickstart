package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BadSSRed2StoneAuto", group="SS")  // @Autonomous(...) is the other common choice
public class SSRed2StoneAuto extends SSOldAutoClasses
{

    int skystone = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initSensors();
        initDogeCV();
        robot.DrivebaseWithEncoders();
        telemetry.addData("Value",runPhoneDetection(0));
        telemetry.update();

        waitForStart();
        runtime.reset();
        skystone = runPhoneDetection(0);
        sleep(150);
        DriveTargetPosition(-1900,0,0,-1900);
        Drive(.5,.5); //.4
        while (robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
        }
        Drive(0,0);

        //Square Wall
        DriveTargetPosition(-75,-75,-75,-75);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
        }
        Drive(0,0);

        DriveTargetPosition(170,170,170,170);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            robot.AutoArmRotate.setPosition(robot.servorotatered);
        }
        Drive(0,0);


//Move to skystone
      StoneMove(skystone,0,200,365);

        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
        }
        Drive(0,0);

        StonePickup();

//Move to Foundation
        StoneMove(skystone,3420,3275,3020);
     Drive(RampPower(3420, maxpower,.3),RampPower(3420, maxpower,.3));
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            telemetry.update();
            Drive(RampPower(3420, maxpower,.3),RampPower(3420, maxpower,.3));
            if (robot.RB.getCurrentPosition() < (robot.RB.getTargetPosition()-100) & robot.RB.getCurrentPosition() > (robot.RB.getTargetPosition()-3000)) {
                robot.AutoArmRotate.setPosition(robot.servorotateback);
            }

            else {
                robot.AutoArmRotate.setPosition(robot.servorotatered);
            }
        }
     Drive(0,0);

       StoneDeliver();

     period.reset();
     PDimu(0,period);

     //Move back for 2nd Skystone
     robot.AutoArmRotate.setPosition(robot.servorotateback);
        StoneMove(skystone,-2630,-2470,-2200);
       Drive(RampPower(-2630, -maxpower,-.35),RampPower(-2630, -maxpower,-.35));
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            telemetry.update();
            Drive(RampPower(-2630, -maxpower,-.3),RampPower(-2630, -maxpower,-.3));
               if(Math.abs(robot.RB.getCurrentPosition() - robot.RB.getTargetPosition())<100){

               }
               else{
                robot.AutoArmRotate.setPosition(robot.servorotateback);}
        }
        Drive(0,0);

        robot.AutoArm.setPosition(robot.servoarmup);
        robot.AutoArmRotate.setPosition(robot.servorotatered);
        robot.AutoArmJoint.setPosition(robot.servojointup);
        sleep(500);
        period.reset();
        PDimu(0,period);

        StonePickup();

        period.reset();
        PDimu(0,period);

        //Move to Foundation
        StoneMove(skystone,2680,2520,2250);
        Drive(RampPower(2630, maxpower,.3),RampPower(2630, maxpower,.3));
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            telemetry.update();
            Drive(RampPower(2630, maxpower,.3),RampPower(2630, maxpower,.3));
            if (robot.RB.getCurrentPosition() < (robot.RB.getTargetPosition()-100) & robot.RB.getCurrentPosition() > (robot.RB.getTargetPosition()-2500)) {
                robot.AutoArmRotate.setPosition(robot.servorotateback);
            }

            else {
                robot.AutoArmRotate.setPosition(robot.servorotatered);
            }
        }
        Drive(0,0);

        StoneDeliver();

        robot.AutoArmJoint.setPosition(robot.servojointup);
        robot.AutoArm.setPosition(robot.servoarmup);
        robot.Grab1.setPosition(robot.grab1home);
        robot.Grab2.setPosition(robot.grab2home);


        DriveTargetPosition(-320,-320,-320,-320);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
          //  liftup();
        }
        Drive(0,0);

        DriveTargetPosition(100,-100,-100,100);
        Drive(.3,.3);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
          //  liftup();

        }
        Drive(0,0);

norm90();
/*while(robot.LiftCurrentPosition()<380){
    robot.Lift(.5);
}
robot.Lift(0);*/
imu(90);



        Drive(.2,.2);
        sleep(1100);
        Drive(0,0);

        robot.LeftLatch.setPosition(robot.leftlatchclose);
        robot.RightLatch.setPosition(robot.rightlatchclose);

        robot.AutoArm.setPosition(robot.servoarmhome);
        robot.AutoArm.setPwmDisable();
        robot.AutoArmJoint.setPwmDisable();

        sleep(1000);
        /*DriveTargetPosition(-1250,-1250,-1250,-1250);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
        }
        Drive(0,0);
        */
        DriveTargetPosition(-320,-320,-320,-320);
        Drive(.4,.4);
        DrivebaseBusy();

        RightAngleTurnTargetPosition(-1400,-1400);
        Drive(0,.4);
        RightDrivebaseBusy();
        Drive(0,0);

        robot.LeftLatch.setPosition(robot.leftlatchopen);
        robot.RightLatch.setPosition(robot.rightlatchopen);

        DriveTargetPosition(100,100,100,100);
        Drive(.4,.4);
        while (robot.LB.isBusy() & robot.RF.isBusy() & robot.LF.isBusy() & robot.RB.isBusy() & opModeIsActive()) {
            //liftup();
        }
        Drive(0,0);

        DriveTargetPosition(-800,800,800,-800);
        Drive(.6,.6);
        DrivebaseBusy();
        Drive(0,0);

        DriveTargetPosition(-500,-500,-500,-500);
        Drive(.4,.4);
        DrivebaseBusy();
        Drive(0,0);

        while(opModeIsActive()) {
            sleep(1000000);
        }

    }
}

