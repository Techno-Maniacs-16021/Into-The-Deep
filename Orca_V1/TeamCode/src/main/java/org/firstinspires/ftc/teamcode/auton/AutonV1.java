package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.OrcaV1;

@Autonomous
@Config
public class AutonV1 extends LinearOpMode {
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double p,i,d,f,target;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        orca = new OrcaV1(hardwareMap,new Pose2d(0,0,0));

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                orca.actionBuilder(new Pose2d(0, 0, 0))
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-11.4,-25.7,Math.toRadians(135)),Math.toRadians(135))
                        .build(),
                orca.deposit(),
                orca.actionBuilder(new Pose2d(-11.4,-25.7,Math.toRadians(135)))
                        .strafeTo(new Vector2d(-8.7,-29))
                        .build(),
                orca.deposit(),
                orca.actionBuilder(new Pose2d(-8.7,-29,Math.toRadians(135)))
                        .strafeTo(new Vector2d(-11.4,-25.7))
                        .build(),
                orca.deposit(),
                orca.actionBuilder(new Pose2d(-11.4,-25.7,Math.toRadians(135)))
                        .splineToSplineHeading(new Pose2d(-54.3,7.7,Math.toRadians(90)),Math.toRadians(270))
                        .build()
        ));

        requestOpModeStop();
    }
}