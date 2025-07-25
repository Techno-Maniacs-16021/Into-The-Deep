package org.firstinspires.ftc.teamcode.auton.imageRec;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class cameraTesting extends OpMode {

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


    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Paths.init();
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
        OrcaV3.autoInit(Paths.startSample,hardwareMap);
        OrcaV3.deposit().sampleInit();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("press cross for blue alliance, press triangle for red alliance");
        telemetry.addData("currently selected alliance: ", OrcaV3.intake().getColorToEject().equals("red") ? "blue" : "red");
        if(gamepad1.cross){
            OrcaV3.intake().setColorToEject("red");
        }
        else if(gamepad1.triangle){
            OrcaV3.intake().setColorToEject("blue");
        }
        OrcaV3.startStream();

        telemetry.update();
    }
    @Override
    public void loop(){
        //OrcaV3.follower().telemetryDebug(telemetryA);
        telemetry.addData("cam_value",OrcaV3.pipeline.getMidX());
        telemetry.update();
    }

    @Override
    public void start() {
        wait.reset();

        OrcaV3.startStream();
        new Wait(3);
        telemetry.addData("cam_value",OrcaV3.pipeline.getMidX());
        telemetry.update();
        new Sequential(
                OrcaV3.retractDeposit(),
//                OrcaV3.startStream(),
//                OrcaV3.setIntake(0.5),
                new Wait(3),
                OrcaV3.setSubIntakeEGAC()
//                OrcaV3.attemptSubIntakeEGAC(),
//                new Wait(intakeWait),
//                OrcaV3.retractIntake()

        ).schedule();
    }
    @Override
    public void stop(){
        OrcaV3.stopStream();
    }

}