package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robots.OrcaV3;

import org.firstinspires.ftc.teamcode.auton.pathing.Paths;

import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Mercurial.Attach
@BulkRead.Attach
@OrcaV3.Attach
@Autonomous
@Config
public class FiveSpec extends OpMode {

    Timer pathTimer;
    String step;
    ElapsedTime wait = new ElapsedTime();


    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Paths.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OrcaV3.autoInit(new Pose(63.5,11,Math.toRadians(0)));
        step = "Init";
    }

    @Override
    public void init_loop() {
    }
    @Override
    public void start(){
        wait.reset();
    }

    @Override
    public void loop() {
        OrcaV3.follower().update();
        auton();
        telemetry.addData("Auton Step: ", step);
        telemetry.update();
    }

    public void auton() {
        if (step.equals("first drop")) {
            new Sequential(
                    OrcaV3.follow(Paths.pathMap.get("firstDeposit-Spec"),true),
                    new Wait(1.0)
            );

        }

    }
}