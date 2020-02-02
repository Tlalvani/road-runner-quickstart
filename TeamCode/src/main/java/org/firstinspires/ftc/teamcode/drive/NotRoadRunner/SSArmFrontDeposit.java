package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="SSArmFrontDeposit", group="SS")  // @Autonomous(...) is the other common choice
public class SSArmFrontDeposit extends SSAutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
       initSensors();
        robot.AutoArm.setPosition(robot.servoarmdown);

        waitForStart();
        StonePickup();
        sleep(1000);
        robot.AutoArm.setPosition(robot.servoarmup);
        sleep(1000);
        robot.AutoArmRotate.setPosition(robot.servorotatehome);
        robot.AutoArmJoint.setPosition(robot.servojointdeliver);
        sleep(1000);
        robot.Grab1.setPosition(robot.grab1home);
        robot.Grab2.setPosition(robot.grab2home);
        sleep(1000);
        ArmUpReset();
        sleep(1000);
        ArmKill();

    }
}

