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
import dev.frozenmilk.mercurial.commands.groups.Race;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Mercurial.Attach
@BulkRead.Attach
@OrcaV3.Attach
//@Autonomous
@Config
public class FiveSample extends OpMode {

    Timer pathTimer;

    private Telemetry telemetryA;

    public PathChain depositChain;
    // = OrcaV3.follower().pathBuilder().addPath()

    ElapsedTime wait = new ElapsedTime();
    public static double intakeWait = 0.4;
    public static double intakeRace = 3.0;
    public static double retractDepositWait = 0.5;
    public static double driveBackWait = 0.2;

    public static double sampleReturnWait = 0.5;

    public static double defaultError = 0.0;

    public static double depositLeave = 0.1;


    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Paths.init();
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        OrcaV3.autoInit(Paths.startSample,hardwareMap);
        OrcaV3.deposit().sampleInit();
        //OrcaV3.setFollowerPower(0.95);
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
        //OrcaV3.follower().telemetryDebug(telemetryA);
    }

    @Override
    public void start() {
        wait.reset();
        new Sequential(
                //STEP: drop first sample (0+1)
                new Parallel(
                        OrcaV3.setSample(true),
                        new Sequential(
                                new Wait(0.2),
                                OrcaV3.follow(Paths.samplePathMap.get("firstDeposit-Sample"),true,defaultError)
                        )
                ),
                OrcaV3.releaseClaw(),
                new Wait(depositLeave),

                //STEP: collect & drop sample (0+2)
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("pick1-Sample"), false,defaultError),
                        new Sequential(
                                //new Wait(0.4),
                                OrcaV3.setIntake(2),
                                new Race(
                                        null,
                                        OrcaV3.attemptIntake(0.75),
                                        new Wait(intakeRace)
                                ),
                                new Wait(intakeWait),
                                OrcaV3.retractIntake()

                        ),
                        new Sequential(
                                new Wait(retractDepositWait),
                                OrcaV3.retractDeposit()
                        )
                ),

                OrcaV3.waitForTransfer(),
                new Parallel(
                        new Sequential(
                                new Wait(driveBackWait),
                                OrcaV3.follow(Paths.samplePathMap.get("drop1-Sample"),true,defaultError)
                        ),
                        OrcaV3.setSample()
                ),
                new Wait(0.2),
                OrcaV3.releaseClaw(),
                new Wait(depositLeave),

                //STEP: collect & drop sample (0+3)
               new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("pick2-Sample"), false,defaultError),
                        new Sequential(
                                new Wait(0.5), //allows intake to get into position
                                OrcaV3.setIntake(1.0),
                                new Race(
                                        null,
                                        OrcaV3.attemptIntake(0.75),
                                        new Wait(intakeRace)
                                ),
                                new Wait(intakeWait),
                                OrcaV3.retractIntake()
                        ),
                        new Sequential(
                                new Wait(retractDepositWait),
                                OrcaV3.retractDeposit()
                        )
               ),

                OrcaV3.waitForTransfer(),
                new Parallel(
                        new Sequential(
                                new Wait(driveBackWait),
                                OrcaV3.follow(Paths.samplePathMap.get("drop2-Sample"),true,defaultError)
                        ),
                        OrcaV3.setSample()
                ),
                new Wait(0.3),
                OrcaV3.releaseClaw(),
                new Wait(depositLeave),

                //STEP: collect & drop sample (0+4)
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("pick3-Sample"), false,defaultError),
                        new Sequential(
                                new Wait(1.0), //allows intake to get into position
                                OrcaV3.setIntake(2),
                                new Race(
                                        null,
                                        OrcaV3.attemptIntake(0.75),
                                        new Wait(intakeRace)
                                ),
                                new Wait(intakeWait),
                                OrcaV3.retractIntake()
                        ),
                        new Sequential(
                                new Wait(retractDepositWait),
                                OrcaV3.retractDeposit()
                        )
                ),
                OrcaV3.waitForTransfer(),
                new Parallel(
                        new Sequential(
                                new Wait(driveBackWait),
                                OrcaV3.follow(Paths.samplePathMap.get("drop3-Sample"),true,defaultError)
                        ),
                        OrcaV3.setSample()
                ),
                new Wait(0.5), //allows deposit to stop swaying
                OrcaV3.releaseClaw(),
                new Wait(depositLeave),

                //OrcaV3.startStream(),
                //STEP: collect from submersible & drop sample (0+5)
              new Parallel(
                      new Sequential(
                              OrcaV3.follow(Paths.samplePathMap.get("collect-Sample"), false,defaultError),
                              //OrcaV3.setIntake(0.5)
                              OrcaV3.setIntake(1.5)
                      ),
                        new Sequential(
                                new Wait(retractDepositWait),
                                OrcaV3.retractDeposit()
                        )
              ),
                //OrcaV3.setSubIntakeEGAC(),
                //OrcaV3.attemptSubIntakeEGAC(),
                OrcaV3.attemptSubIntake(),
                new Wait(intakeWait),
                OrcaV3.retractIntake(),
                new Parallel(
                        new Sequential(
                                new Wait(sampleReturnWait),
                                OrcaV3.follow(Paths.samplePathMap.get("deposit-Sample"),true,defaultError)
                        ),
                        new Sequential(
                                OrcaV3.waitForTransfer(),
                                OrcaV3.setSample()
                        )
                ),
                OrcaV3.releaseClaw(),

                //STEP: park at submersible
                new Parallel(
                        new Sequential(
                                OrcaV3.follow(Paths.samplePathMap.get("collect-Sample2"), false,defaultError)
                                //OrcaV3.setIntake(0.5)
                                //OrcaV3.setIntake(1.5)
                        ),
                        new Sequential(
                                new Wait(retractDepositWait),
                                OrcaV3.retractDeposit()
                        )
                )
                //OrcaV3.setSubIntakeEGAC(),
                //OrcaV3.attemptSubIntakeEGAC(),
                /*OrcaV3.attemptSubIntake(),
                new Wait(intakeWait),
                OrcaV3.retractIntake(),
                new Parallel(
                        new Sequential(
                                new Wait(sampleReturnWait),
                                OrcaV3.follow(Paths.samplePathMap.get("deposit-Sample2"),true,defaultError)
                        ),
                        new Sequential(
                                OrcaV3.waitForTransfer(),
                                OrcaV3.setSample()
                        )
                ),
                OrcaV3.releaseClaw(),

                //end auto
                new Parallel(
                        OrcaV3.follow(Paths.samplePathMap.get("park-Sample"), false,defaultError),
                        new Sequential(
                                new Wait(retractDepositWait),
                                OrcaV3.retractDeposit()
                        )
                )*/

        ).schedule();
    }

    public void stop() {
        //OrcaV3.stopStream();
    }

}