package org.firstinspires.ftc.teamcode.drive.NotRoadRunner;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 *
 * LB = Left Back
 * LF = Left Front
 * RB = Right Back
 * RF = Right Front
 */


@TeleOp(name = "SSTeleop", group = "SS")  // @Autonomous(...) is the other common choice
public class SSTeleop extends OpMode {

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();
    SSHardwareDrivebase robot = new SSHardwareDrivebase();
    double currentVelocity;
    double maxVelocity = 0.0;
    boolean intakearmout = true;
    boolean intakearmfat = false;
    double i=0;
    /* Constructor */
    @Override
    public void init() {

        robot.init(hardwareMap);


      //  robot.DrivebaseWithEncoders();


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        runtime.reset();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.Grab1.setPosition(robot.grab1home);
        robot.Grab2.setPosition(robot.grab2home);

        if(gamepad1.right_bumper||gamepad1.x){
            //OVERRIDE
        }
        else if(intakearmfat){
            robot.RightIntakeArm.setPosition(robot.rightintakearmfat);
            robot.LeftIntakeArm.setPosition(robot.leftintakearmfat);
        }
        else if(intakearmout){
            robot.RightIntakeArm.setPosition(robot.rightintakearmout);
            robot.LeftIntakeArm.setPosition(robot.leftintakearmout);
        }
        else{
            robot.RightIntakeArm.setPosition(robot.rightintakearmin);
            robot.LeftIntakeArm.setPosition(robot.leftintakearmin);
        }

if(gamepad1.back){
    robot.ParkSlideOut();
}
else{
    robot.ParkSlideIn();
}
/*
        if(runtime.seconds()>122){
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
        }
else if(runtime.seconds()>110){
    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
}

else if(runtime.seconds()>90){
    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

}
else if (runtime.seconds()>60){
    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        }
else{
    robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
}*/
        //robot.AutoArm.setPosition(robot.servoarmhome);

        float Ch1 = (gamepad1.right_stick_x);
        float Ch3 = -gamepad1.left_stick_y;
        float Ch4 = gamepad1.left_stick_x;


        float rightfront = Ch3 - Ch1 - Ch4;
        float rightback = Ch3 - Ch1 + Ch4;
        float leftfront = Ch3 + Ch1 + Ch4;
        float leftback = Ch3 + Ch1 - Ch4;
        float lift = gamepad1.right_trigger-gamepad1.left_trigger;



        rightback = Range.clip(rightback, -1, 1);
        leftback = Range.clip(leftback, -1, 1);
        rightfront = Range.clip(rightfront, -1, 1);
        leftfront = Range.clip(leftfront, -1, 1);
        lift = Range.clip(lift, -1, 1);



        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        rightfront = (float) robot.scaleInput(rightfront);
        leftfront = (float) robot.scaleInput(leftfront);
        rightback = (float) robot.scaleInput(rightback);
        leftback = (float) robot.scaleInput(leftback);


        // write the values to the motors
        robot.RF.setPower(rightfront);
        robot.LF.setPower(leftfront);
        robot.RB.setPower(rightback);
        robot.LB.setPower(leftback);

        robot.Lift1.setPower(lift);
        robot.Lift2.setPower(lift);

        if(gamepad1.right_trigger>.1){
            intakearmout = true;
        }

        if(gamepad1.right_bumper){
            robot.LeftIntake.setPower(.75);
            robot.RightIntake.setPower(.75);

            robot.RightIntakeArm.setPosition(robot.rightintakearmin);
            robot.LeftIntakeArm.setPosition(robot.leftintakearmin);

            robot.LeftClaw.setPosition(robot.leftclawopen);
            robot.RightClaw.setPosition(robot.rightclawclose);

            intakearmfat = false;
        }

        else if (gamepad1.x){
            robot.LeftIntake.setPower(-.75);
            robot.RightIntake.setPower(-.75);

            robot.RightIntakeArm.setPosition(robot.rightintakearmin);
            robot.LeftIntakeArm.setPosition(robot.leftintakearmin);
            intakearmfat = false;
        }

        else if(gamepad1.dpad_right){
            intakearmfat = true;
        }
        else{
            robot.LeftIntake.setPower(0);
            robot.RightIntake.setPower(0);



        }

        if(gamepad1.left_bumper){
            robot.LeftClaw.setPosition(robot.leftclawclose);
            robot.RightClaw.setPosition(robot.rightclawclose);
            intakearmout = false;
        }

        if(gamepad1.a){
            robot.LeftClaw.setPosition(robot.leftclawopen);
            intakearmout = true;
        }

        if(gamepad1.y){
robot.LeftClaw.setPosition(robot.gripperup);
        }
        if(gamepad1.dpad_up){
            robot.RightClaw.setPosition(robot.rightclawopen);
        }

        if(gamepad1.dpad_down){
            robot.RightClaw.setPosition(robot.rightclawclose);

        }

        if(gamepad1.right_stick_button){
            robot.LeftLatch.setPosition(robot.leftlatchclose);
            robot.RightLatch.setPosition(robot.rightlatchclose);
        }
        if(gamepad1.left_stick_button){
            robot.LeftLatch.setPosition(robot.leftlatchopen);
            robot.RightLatch.setPosition(robot.rightlatchopen);
        }

        if(gamepad2.a){
            robot.AutoArm.setPwmDisable();
        }
        else if(gamepad2.y){
            robot.AutoArmRotate.setPosition(robot.servorotateback);
            robot.AutoArm.setPwmEnable();
            robot.AutoArm.setPosition(robot.servoarmup);

        }
        else{
            robot.AutoArmRotate.setPwmEnable();
            robot.AutoArmRotate.setPosition(robot.servorotatehome);
        }

        telemetry.addData("LF: ", robot.LF.getCurrentPosition());
        //telemetry.addData("looptime",i++);
       /* telemetry.addData("LB: ", robot.LB.getCurrentPosition());
        telemetry.addData("RF: ", robot.RF.getCurrentPosition());
        telemetry.addData("RB: ", robot.RB.getCurrentPosition());
        telemetry.addData("Velocity: ", maxVelocity);
        telemetry.addData("Right", robot.Lift2.getCurrentPosition());
        telemetry.addData("Left", robot.LeftIntake.getCurrentPosition());
        telemetry.addData("Strafe", robot.RightIntake.getCurrentPosition()); */

        telemetry.update();

    }

    @Override
    public void stop() {
    }
}







