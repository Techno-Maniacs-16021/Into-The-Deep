package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.WeakReferenceSet;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robots.OrcaV3;

import org.firstinspires.ftc.teamcode.auton.pathing.Paths;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.UnitComponent;
import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Mercurial.Attach
@BulkRead.Attach
@OrcaV3.Attach
@Autonomous
@Config
public class FiveSample extends OpMode {

    Timer pathTimer;

    private Telemetry telemetryA;

    public PathChain depositChain;
    // = OrcaV3.follower().pathBuilder().addPath()

    ElapsedTime wait = new ElapsedTime();


    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Paths.init();
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        OrcaV3.autoInit(Paths.startSample,hardwareMap);
    }

    @Override
    public void init_loop() {
    }
    @Override
    public void loop(){
        OrcaV3.follower().telemetryDebug(telemetryA);
    }

    @Override
    public void start() {
        wait.reset();
        new Sequential(
                //STEP: drop first sample (0+1)
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("firstDeposit-Sample"),true),
                        OrcaV3.setSample()
                ),
                OrcaV3.releaseClaw(),

                //STEP: collect & drop sample (0+2)
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("pick1-Sample"), false),
                        OrcaV3.retractDeposit()
                        //intake sample
                ),
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("drop1-Sample"),true),
                        OrcaV3.setSample()
                ),
                OrcaV3.releaseClaw(),

                //STEP: collect & drop sample (0+3)
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("pick2-Sample"), false),
                        OrcaV3.retractDeposit()
                        //intake sample
                ),
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("drop2-Sample"),true),
                        OrcaV3.setSample()
                ),
                OrcaV3.releaseClaw(),

                //STEP: collect & drop sample (0+4)
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("pick3-Sample"), false),
                        OrcaV3.retractDeposit()
                        //intake sample
                ),
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("drop3-Sample"),true),
                        OrcaV3.setSample()
                ),
                OrcaV3.releaseClaw(),

                //STEP: collect from submersible & drop sample (0+5)
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("collect-Sample"), false),
                        OrcaV3.retractDeposit()
                        //intake sample
                ),
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("deposit-Sample"),true),
                        OrcaV3.setSample()
                ),
                OrcaV3.releaseClaw(),
                OrcaV3.retractDeposit()

        ).schedule();
    }

}