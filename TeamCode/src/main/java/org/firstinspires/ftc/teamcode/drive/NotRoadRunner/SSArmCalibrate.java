package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="SSArmCalibrate", group="SS")  // @Autonomous(...) is the other common choice
public class SSArmCalibrate extends SSAutoClasses
{




    @Override
    public void runOpMode() throws InterruptedException {
       initSensors();
        waitForStart();
        while(opModeIsActive()) {
if(gamepad1.dpad_right){
robot.AutoArmRotate.setPosition(robot.servorotaterblue);}

            if(gamepad1.dpad_left){
                robot.AutoArmRotate.setPosition(robot.servorotatered);}

            if(gamepad1.dpad_down){
                robot.AutoArmRotate.setPosition(robot.servorotateback);}

            if(gamepad1.dpad_up){
                robot.AutoArmRotate.setPosition(robot.servorotatehome);}

if(gamepad1.a){
    robot.AutoArm.setPosition(robot.servoarmdown);

}
if(gamepad1.y){
    robot.AutoArm.setPosition(robot.servoarmpickup);
}

if(gamepad1.x){
    robot.AutoArm.setPosition(.15);
}
        }

    }
}

