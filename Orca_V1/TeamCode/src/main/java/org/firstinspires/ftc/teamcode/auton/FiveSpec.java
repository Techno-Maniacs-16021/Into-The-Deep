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
    public static double grabWait = 0.0;
    public static double leaveWait = 0.25;
    public static double retractWait = 0 ; //this is good

    public static double defaultError = 1.5;

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
        telemetry.addLine("press cross for blue alliance, press triangle for red alliance");
        telemetry.addData("currently selected alliance: ", OrcaV3.intake().getColorToEject().equals("red") ? "blue" : "red");
        if(gamepad1.cross||gamepad2.cross){
            OrcaV3.intake().setColorToEject("red");
        }
        else if(gamepad1.triangle||gamepad2.triangle){
            OrcaV3.intake().setColorToEject("blue");
        }
        telemetry.update();
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
                        OrcaV3.follow(Paths.specPathMap.get("firstDeposit-Spec"),true,defaultError),
                        OrcaV3.setSpecimen()
                ),
                new Parallel(
                        new Wait(0.25),
                        OrcaV3.releaseClaw()
                ),
                new Parallel(
                        //OrcaV3.follow(Paths.specPathMap.get("pick1-Spec"), false),
                        OrcaV3.follow(Paths.specPathMap.get("pick1-Spec"), Paths.specPathMap.get("drop1-Spec"),5, false,defaultError),
                        OrcaV3.retractSpecimenDeposit()
                ),
                OrcaV3.follow(Paths.specPathMap.get("pick2-Spec"), Paths.specPathMap.get("drop2-Spec"),5,false,defaultError),
                OrcaV3.follow(Paths.specPathMap.get("pick3-Spec"), Paths.specPathMap.get("drop3-Spec"),5,false,defaultError),
                OrcaV3.follow(Paths.specPathMap.get("firstCollect-Spec"),false,defaultError,5),

                //STEP: collect 3 specs & ready for intake

                //STEP: pickup spec
                //OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false),
                new Wait(grabWait),
                OrcaV3.closeClaw(),
                new Wait(leaveWait),

                //STEP: deposit spec (2+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("deposit-Spec2"), true,defaultError),
                        OrcaV3.setSpecimen()
                ),

                //STEP: go to collection
                new Parallel(
                        new Sequential(
                                OrcaV3.follow(Paths.specPathMap.get("align-Spec-Curve"), false,4*defaultError),
                                OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false,defaultError,6)
                                //OrcaV3.follow(Paths.specPathMap.get("align-Spec-Curve"),Paths.specPathMap.get("collect-Spec"),false,defaultError)
                        ),
                        OrcaV3.retractSpecimenDeposit()
                ),

                //STEP: pickup spec
                //OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false),
                new Wait(grabWait),
                OrcaV3.closeClaw(),
                new Wait(leaveWait),

                //STEP: deposit spec (3+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("deposit-Spec3"), true,defaultError),
                        OrcaV3.setSpecimen()
                ),

                //STEP: go to collection
                new Parallel(
                        new Sequential(
                                OrcaV3.follow(Paths.specPathMap.get("align-Spec-Curve"), false,4*defaultError),
                                OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false,defaultError,6)
                                //OrcaV3.follow(Paths.specPathMap.get("align-Spec-Curve"),Paths.specPathMap.get("collect-Spec"),false,defaultError)
                        ),
                        OrcaV3.retractSpecimenDeposit()
                ),

                //STEP: pickup spec
                //OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false),
                new Wait(grabWait),
                OrcaV3.closeClaw(),
                new Wait(leaveWait),

                //STEP: deposit spec (4+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("deposit-Spec4"), true,defaultError),
                        OrcaV3.setSpecimen()
                ),

                //STEP: go to collection
                new Parallel(
                        new Sequential(
                                OrcaV3.follow(Paths.specPathMap.get("align-Spec-Curve"), false,4*defaultError),
                                OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false,defaultError,6)
                                //OrcaV3.follow(Paths.specPathMap.get("align-Spec-Curve"),Paths.specPathMap.get("collect-Spec"),false,defaultError)
                        ),
                        OrcaV3.retractSpecimenDeposit()
                ),

                //STEP: pickup spec
                //OrcaV3.follow(Paths.specPathMap.get("collect-Spec"), false),
                new Wait(grabWait),
                OrcaV3.closeClaw(),
                new Wait(leaveWait),

                //STEP: deposit spec (5+0)
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("deposit-Spec5"), true,defaultError),
                        OrcaV3.setSpecimen()
                ),

                //STEP: park
                new Parallel(
                        OrcaV3.follow(Paths.specPathMap.get("specPark"), false,defaultError),
                        new Sequential(
                                OrcaV3.releaseClaw(),
                                //new Wait(retractWait),
                                OrcaV3.retractSpecimenDepositFinal()
                        )
                )

        ).schedule();
    }

}