package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="SSParkAuto", group="SS")  // @Autonomous(...) is the other common choice
public class SSParkAuto extends SSAutoClasses
{
    ElapsedTime loopTime = new ElapsedTime();





    @Override
    public void runOpMode() throws InterruptedException {
       initSensors();
       robot.AutoArm.setPwmDisable();
        robot.Grab1.setPosition(robot.grab1home);
        robot.Grab2.setPosition(robot.grab2home);
        waitForStart();

        loopTime.reset();

        robot.RightIntake.setPower(1);
        robot.LeftIntake.setPower(1);
        sleep(2000);
        robot.RightIntake.setPower(0);
        robot.LeftIntake.setPower(0);
        robot.LeftClaw.setPosition(robot.leftclawopen);
    }
}

