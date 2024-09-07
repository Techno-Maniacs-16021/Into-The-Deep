package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BasicRobot;
import org.firstinspires.ftc.teamcode.robots.Robot;

public class DriveTeleOp extends LinearOpMode {
    public BasicRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BasicRobot();
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            robot.driveTrain.drive(drive, turn);
            telemetry.update();
        }

        robot.stop();
    }
}
