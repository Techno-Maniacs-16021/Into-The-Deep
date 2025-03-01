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

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.mercurial.Mercurial;
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
    ElapsedTime wait = new ElapsedTime();


    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Paths.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OrcaV3.autoInit(new Pose(63.5,11,Math.toRadians(0)));
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
        new Sequential(
                //STEP: drop first spec
                new Parallel(
                        OrcaV3.follow(Paths.pathMap.get("firstDeposit-Spec"),true),
                        OrcaV3.setSpecimen()
                ),
                //@Rick: release claw

                //STEP: collect 3 specs & ready for intake
                new Parallel(
                        new Sequential(
                                OrcaV3.follow(Paths.pathMap.get("pick1-Spec"), false),
                                OrcaV3.follow(Paths.pathMap.get("pick1-Spec"), false),
                                OrcaV3.follow(Paths.pathMap.get("drop1-Spec"), false),
                                OrcaV3.follow(Paths.pathMap.get("pick2-Spec"), false),
                                OrcaV3.follow(Paths.pathMap.get("drop2-Spec"), false),
                                OrcaV3.follow(Paths.pathMap.get("pick3-Spec"), false),
                                OrcaV3.follow(Paths.pathMap.get("drop3-Spec"), false),
                                OrcaV3.follow(Paths.pathMap.get("firstAlign-Spec"), false),
                                new Wait(0.5)
                        )
                        //@Rick: ready for spec intake
                ),

                //STEP: pickup spec
                OrcaV3.follow(Paths.pathMap.get("collect-Spec"), true),
                new Wait(0.5),
                //@Rick: close claw
                new Wait(0.25),

                //STEP: deposit spec
                new Parallel(
                        OrcaV3.follow(Paths.pathMap.get("deposit-Spec"), true),
                        OrcaV3.setSpecimen()
                ),
                //@Rick: release claw

                //STEP: go to collection
                new Parallel(
                        OrcaV3.follow(Paths.pathMap.get("align-Spec"), false)//,
                        //@Rick: ready for spec intake
                ),

                //STEP: pickup spec
                OrcaV3.follow(Paths.pathMap.get("collect-Spec"), true),
                new Wait(0.5),
                //@Rick: close claw
                new Wait(0.25)

        );
        //is .schedule() needed?
    }
}