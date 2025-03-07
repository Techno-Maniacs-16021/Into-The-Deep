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
public class FiveSpec extends OpMode {

    Timer pathTimer;
    public double grabWait = 0.0;
    public double leaveWait = 0.0;
    public double retractWait = 0.25; //this is good

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
        OrcaV3.autoInit(Paths.startSpec,hardwareMap);
        OrcaV3.deposit().specInit();
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
                //STEP: drop first spec (1+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("firstDeposit-Spec"),true),
                        OrcaV3.setSpecimen()
                ),
                new Parallel(
                        //OrcaV3.follow(Paths.specPathMap.get("pick1-Spec"), false),
                        OrcaV3.follow(Paths.specPathMap.get("pick1-Spec"), Paths.specPathMap.get("drop1-Spec"),6, false),
                        OrcaV3.retractSpecimenDeposit()
                ),
                OrcaV3.follow(Paths.specPathMap.get("pick2-Spec"), Paths.specPathMap.get("drop2-Spec"),6,false),
                OrcaV3.follow(Paths.specPathMap.get("pick3-Spec"), Paths.specPathMap.get("drop3-Spec"), 6,false),

                OrcaV3.follow(Paths.specPathMap.get("firstCollect-Spec"),false),

                //STEP: collect 3 specs & ready for intake

                //STEP: pickup spec
                //OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false),
                new Wait(grabWait),
                OrcaV3.closeClaw(),
                new Wait(leaveWait),

                //STEP: deposit spec (2+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("deposit-Spec2"), true),
                        OrcaV3.setSpecimen()
                ),

                //STEP: go to collection
                new Parallel(
                        //OrcaV3.follow(Paths.specPathMap.get("align-Spec"), false),
                        OrcaV3.follow(Paths.specPathMap.get("collectNoAlign-Spec"), false),
                        OrcaV3.retractSpecimenDeposit()
                ),

                //STEP: pickup spec
                //OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false),
                new Wait(grabWait),
                OrcaV3.closeClaw(),
                new Wait(leaveWait),

                //STEP: deposit spec (3+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("deposit-Spec3"), true),
                        OrcaV3.setSpecimen()
                ),

                //STEP: go to collection
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("collectNoAlign-Spec"), false),
                        //OrcaV3.follow(Paths.specPathMap.get("align-Spec"), false),
                        OrcaV3.retractSpecimenDeposit()
                ),

                //STEP: pickup spec
                //OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false),
                new Wait(grabWait),
                OrcaV3.closeClaw(),
                new Wait(leaveWait),

                //STEP: deposit spec (4+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("deposit-Spec4"), true),
                        OrcaV3.setSpecimen()
                ),

                //STEP: go to collection
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("collectNoAlign-Spec"), false),
                        //OrcaV3.follow(Paths.specPathMap.get("align-Spec"), false),
                        OrcaV3.retractSpecimenDeposit()
                ),

                //STEP: pickup spec
                //OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false),
                new Wait(grabWait),
                OrcaV3.closeClaw(),
                new Wait(leaveWait),

                //STEP: deposit spec (5+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("deposit-Spec5"), true),
                        OrcaV3.setSpecimen()
                ),

                //STEP: park
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("align-Spec"), false),
                        new Sequential(
                                OrcaV3.releaseClaw(),
                                new Wait(retractWait),
                                OrcaV3.retractDeposit()
                        )
                )

        ).schedule();
    }

}