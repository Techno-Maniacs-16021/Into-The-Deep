package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;

import android.service.quickaccesswallet.SelectWalletCardRequest;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robots.OrcaV2;

import org.firstinspires.ftc.teamcode.auton.pathing.Paths;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.features.BulkRead;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;

@Mercurial.Attach
@BulkRead.Attach
@Autonomous
@Config
public class test extends OpMode {

    OrcaV2 orca;
    Timer pathTimer;
    int step;
    int cycles;
    ElapsedTime wait = new ElapsedTime();


    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        orca = new OrcaV2(hardwareMap, new Pose(63.5,11,Math.toRadians(0)));
        Paths.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        orca.autoInit();
        step = 0;
        cycles = 0;
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
        orca.getFollower().update();
        auton();
        telemetry.addData("Auton Step", step);
        telemetry.addData("Position", orca.getFollower().getPose().toString());
        telemetry.update();
    }

    public void auton() {
        switch(step) {
            case 0: //Deposit spec
                orca.getFollower().followPath(Paths.pathList.get(0), true);
                nextStep(1000);
                break;
            case 1000:
                orca.deposit().depositSpecimen();
                orca.deposit().refresh();
                if(orca.deposit().slidesReachedTarget()){
                    nextStep(10);
                }
                break;
            case 1: //pickup 1
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(1), true);
                    nextStep(1001);
                }
                break;
            case 1001:
                orca.deposit().specimenRetract();
                orca.deposit().refresh();
                if (orca.deposit().slidesReachedTarget()){
                    nextStep(2);
                }
                break;
            case 2: //dropoff 1
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(2), true);
                    nextStep(3);
                }
                break;

            case 3: //pickup 2
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(3), true);
                    nextStep(4);
                }
                break;

            case 4: //dropoff 2
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(4), true);
                    nextStep(5);
                }
                break;

            case 5: //pickup 3
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(5), true);
                    nextStep(7);
                }
                break;
            case 6: //dropoff 3
                if (!orca.getFollower().isBusy()) {
                    orca.turnTo(Math.toRadians(0));
                    nextStep(7);
                }
                break;
            case 7: //spec collect
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(6), true);
                    nextStep(1007);
                }
                break;
            case 1007:
                orca.deposit().specimenIntake();
                orca.deposit().refresh();
                if(orca.deposit().slidesReachedTarget()){
                    wait.reset();
                    nextStep(11);
                }
                break;
            case 8: //spec drop (cycle)
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(8), true);
                    nextStep(1008);
                }
                break;
            case 1008:
                orca.deposit().depositSpecimen();
                orca.deposit().refresh();
                if(orca.deposit().slidesReachedTarget()) {
                    nextStep(10);
                }
                break;
            case 9: //spec collect (cycle)
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(9), true);
                    nextStep(2009);
                }
                break;
            case 2009:
                if (!orca.getFollower().isBusy()) {
                    orca.getFollower().followPath(Paths.pathList.get(6), true);
                    nextStep(1009);
                }
                break;
            case 1009:
                orca.deposit().specimenIntake();
                orca.deposit().refresh();
                if(orca.deposit().slidesReachedTarget()){
                    if (cycles == 6) {
                        nextStep(-1);
                    }
                    else {
                        nextStep(11);
                    }
                }
                break;
            case 10:
                if (!orca.getFollower().isBusy()) {
                    orca.deposit().clipSpecimen();
                    orca.deposit().refresh();
                    if(orca.deposit().slidesReachedTarget()){
                        if (cycles == 0) {
                            cycles++;
                            nextStep(1);
                        }
                        else {
                            cycles++;
                            nextStep(9);
                        }
                    }

                }
                break;
            case 11:
                if (!orca.getFollower().isBusy()) {
                    if(wait.milliseconds()>500){
                        orca.deposit().closeClaw();
                        orca.deposit().refresh();
                    }

                    if(wait.milliseconds()>1000){
                        nextStep(8);
                    }
                }
                break;
            case -1:
                requestOpModeStop();
                break;

        }
    }

    public void nextStep(int num) {
        step = num;
        pathTimer.resetTimer();
    }
}