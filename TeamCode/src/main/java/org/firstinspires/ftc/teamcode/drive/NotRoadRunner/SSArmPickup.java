package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="SSArmPickup", group="SS")  // @Autonomous(...) is the other common choice
public class SSArmPickup extends SSAutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
       initSensors();
        robot.AutoArm.setPosition(robot.servoarmdown);

        waitForStart();
        robot.AutoArmRotate.setPosition(robot.servorotaterblue);
        sleep(2000);
        StonePickup();
sleep(1000000);

    }
}

