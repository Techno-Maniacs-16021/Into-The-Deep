package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robots.OrcaV2;

import org.firstinspires.ftc.teamcode.auton.pathing.Paths;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
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
        //orca = new OrcaV2(hardwareMap, new Pose(0,0,Math.toRadians(0)));
        Paths.init();
        pathTimer = new Timer();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        orca.refresh();
        telemetry.addData("Position", orca.getFollower().getPose().toString());
        telemetry.update();
    }

    @Override
    public void start() {
        new Sequential(
                new Parallel( //SPECIMEN DROP
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
        ).schedule();
    }
}