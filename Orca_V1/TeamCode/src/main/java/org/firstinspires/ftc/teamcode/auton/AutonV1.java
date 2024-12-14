package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.localization.Drawing;
import org.firstinspires.ftc.teamcode.robots.OrcaV1;
import org.firstinspires.ftc.teamcode.auton.AutonBase;

@Autonomous
@Config
public class AutonV1 extends LinearOpMode {
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double heading = -45;
    public static int targetx,targety,targeth;

    Vector2d goUpPlace = new Vector2d(-11.36,-36.59);
    Pose2d depositPlace = new Pose2d(-8.3,-40.2,Math.toRadians(135));
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //orca = initRobot(0,0,0);
        orca = new OrcaV1(hardwareMap,new Pose2d(0,0,0));
        waitForStart();
        runningTime.reset();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), orca.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                orca.actionBuilder(new Pose2d(0, 0, 0))
                        .setTangent(Math.toRadians(180))
                        .splineTo(goUpPlace,Math.toRadians(heading))
                        .build()
                //getArmToGround(orca),
                //orca.deposit(),
//                orca.actionBuilder(new Pose2d(-11.4,-25.7,Math.toRadians(135)))
//                        .strafeTo(new Vector2d(-8.7,-29))
//                        .build(),
//                //orca.deposit(),
//                orca.actionBuilder(new Pose2d(-8.7,-29,Math.toRadians(135)))
//                        .strafeTo(new Vector2d(-11.4,-25.7))
//                        .build(),
//                //orca.deposit(),
//                orca.actionBuilder(new Pose2d(-11.4,-25.7,Math.toRadians(135)))
//                        .splineToSplineHeading(new Pose2d(-54.3,7.7,Math.toRadians(90)),Math.toRadians(270))
//                        .build()
        ));



        requestOpModeStop();
    }
}