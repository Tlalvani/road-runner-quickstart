package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="SSResetArm", group="SS")  // @Autonomous(...) is the other common choice
public class SSResetArm extends SSAutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
       initSensors();
        waitForStart();

        while(opModeIsActive()) {

robot.AutoArmJoint.setPosition(robot.servojointup);
robot.AutoArm.setPosition(robot.servoarmup);
robot.Grab1.setPosition(robot.grab1home);
robot.Grab2.setPosition(robot.grab2home);
sleep(2000);
robot.AutoArmRotate.setPosition(robot.servorotatehome);
sleep(2000);
robot.AutoArm.setPosition(robot.servoarmhome);
robot.AutoArm.setPwmDisable();
robot.AutoArmJoint.setPwmDisable();


//robot.AutoArmJoint.setPosition(robot.servojointdown);
//robot.AutoArmRotate.setPosition(robot.servorotaterblue);

            sleep(1000000);
        }

    }
}

