package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathingOld.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathingOld.constants.LConstants;

import org.firstinspires.ftc.teamcode.auton.pathing.Paths;
import org.firstinspires.ftc.teamcode.pedroPathingOld.localization.Pose;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.localization.Drawing;
import org.firstinspires.ftc.teamcode.robots.OrcaV2;

import java.util.Timer;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

@Mercurial.Attach
@BulkRead.Attach
@Autonomous
@Config
public class FiveSpec extends OpMode {

    OrcaV2 orca;
    Timer pathTimer;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        orca = new OrcaV2(hardwareMap, new Pose(-65.5,-11,Math.toRadians(0)));
        Paths.init();
        pathTimer = new Timer();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        orca.update();

        new Sequential(
                orca.follow(Paths.pathList.get(0))
                /*orca.follow(Paths.pathList.get(1)),
                orca.turnTo(Math.toRadians(-40.3)),
                orca.follow(Paths.pathList.get(2)),
                orca.turnTo(Math.toRadians(-35)),
                orca.follow(Paths.pathList.get(3)),
                orca.follow(Paths.pathList.get(4)),
                orca.turnTo(Math.toRadians(90)),
                orca.follow(Paths.pathList.get(5)),
                orca.follow(Paths.pathList.get(6)),
                orca.follow(Paths.pathList.get(7))*/
        ).schedule();

        telemetry.addData("Position", orca.getPose().toString());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), new Pose2d(orca.getPose().getX(),orca.getPose().getY(),orca.getPose().getHeading()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void start() {
        /*new Sequential(
                new Parallel ( //SPECIMEN DROP
                        orca.follow(Paths.pathList.get(0)),
                        new Sequential(
                                //set specimen + smart wait + deposit specimen
                        )
                ),
                new Parallel( //PICKUP 1
                        orca.follow(Paths.pathList.get(1))
                        //intake ground
                        //retract deposit
                ),
                new Parallel( //DROPOFF 1
                        orca.turnTo(Math.toRadians(-40.3)),
                        new Sequential(
                                //smart wait + outtake ground
                        )
                ),
                new Parallel( //PICKUP 2
                        orca.follow(Paths.pathList.get(2))
                        //set intake

                ),
                //intake ground
                new Parallel( //DROPOFF 2
                        orca.turnTo(Math.toRadians(-35)),
                        new Sequential(
                                //smart wait + outtake ground
                        )
                ),
                new Parallel( //PICKUP 3
                        orca.follow(Paths.pathList.get(3))
                        //set intake
                ),
                //intake ground
                new Parallel( //DROP OFF 3 + COLLECT SPECIMEN
                        orca.follow(Paths.pathList.get(4)),
                        orca.turnTo(Math.toRadians(90)),
                        orca.follow(Paths.pathList.get(5)),
                        new Sequential(
                                //set intake
                                //smart wait
                                //outtake ground
                                //retract intake
                                //ready pickup spec
                                //smart wait
                                //pickup spec
                                //smart wait
                        )
                ),
                new Parallel( //SPECIMEN DROP
                        orca.follow(Paths.pathList.get(6)),
                        new Sequential(
                                //set specimen
                                //smart wait
                                //deposit specimen
                        )
                ),
                new Parallel( //COLLECT SPECIMEN
                        orca.follow(Paths.pathList.get(7)),
                        new Sequential(
                                //retract deposit
                                //ready pickup spec
                                //smart wait
                                //pickup spec
                                //smart wait
                        )
                )
        ).schedule();*/
    }
}