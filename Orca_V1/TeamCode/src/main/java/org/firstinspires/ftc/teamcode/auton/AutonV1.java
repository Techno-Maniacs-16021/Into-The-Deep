package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.localization.Drawing;
import org.firstinspires.ftc.teamcode.robots.OrcaV1;
import org.firstinspires.ftc.teamcode.auton.AutonBase;

@Autonomous
@Config
public class AutonV1 extends LinearOpMode {
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double x1,y1,h1,x2;
    // -28 --> -30.6, 5.7

    Vector2d goUpPlace = new Vector2d(-11.36,-36.59);
    Pose2d depositPlace = new Pose2d(-8.3,-40.2,Math.toRadians(135));
    Vector2d preSpecimen = new Vector2d(x1,y1);
    Vector2d finalSpecimen = new Vector2d(x2,y1);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //orca = initRobot(0,0,0);
        orca = new OrcaV1(hardwareMap,new Pose2d(0,0,0));
        orca.deposit().specimenIntake();
        orca.deposit().refresh();
        waitForStart();
        runningTime.reset();
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("Voltage: "+orca.intake().getRotationVoltage());
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), orca.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
        Actions.runBlocking(new SequentialAction(
                orca.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeTo(preSpecimen)
                        .build(),
                intakeSampleGround(orca,0),
                setSpecimen(orca),
                orca.actionBuilder(new Pose2d(preSpecimen.x,preSpecimen.y,0))
                        .strafeTo(finalSpecimen)
                        .build(),
                depositSpecimen(orca)

                //getArmToGround(orca),
                //orca.deposit(),
//                orca.actionBuilder(new Pose2d(-11.4,-25.7,Math.toRadians(135)))
//                        .strafeTo(new Vector2d(-8.7,-29))
//                        .build(),
//                //orca.deposit(),
//                orca.actionBuilder(new Pose2d(-8.7,-29,Math.toRadians(135)))
//                        .strafeTo(new Vector2d(-11.4,-25.7))
//                        .build(),
//                //orca.deposit(),
//                orca.actionBuilder(new Pose2d(-11.4,-25.7,Math.toRadians(135)))
//                        .splineToSplineHeading(new Pose2d(-54.3,7.7,Math.toRadians(90)),Math.toRadians(270))
//                        .build()
        ));



        requestOpModeStop();
    }
    public Action orcaRefresh(OrcaV1 bot , double slidesPower, boolean verticalIntake, boolean angledIntake, boolean reverseIntake, boolean retract, boolean pid ) {
        return telemetryPacket -> {
            //TODO: Use this anytime the mechanisms are moving while the robot is moving in a parallel action
            bot.deposit().refresh();
            bot.intake().refresh(slidesPower,verticalIntake,angledIntake,reverseIntake,retract,pid);
            return false;
        };
    }
    public Action setSample(OrcaV1 bot ) {
        //TODO: YOU CAN USE THESE(setSample or setSpecimen) TO GET READY TO SCORE
        //Basically it will raise the slides to the right height.
        //Lets say your transfer is done, and your are moving to the bucket,
        //you can use this action in a parallel action with the movement
        //to have the slides raise up while traveling to the position
        //the depositSample and depositSpecimen Actions will reach the target and deposit imediately
        return telemetryPacket -> {
            if(bot.intake().getIntakeCommand().equals("transfer")){
               return true;
            }
            bot.deposit().setSample();
            bot.deposit().refresh();
            return false;
        };
    }
    public Action setSpecimen(OrcaV1 bot ) {
        return telemetryPacket -> {
            bot.deposit().setSpecimen();
            bot.deposit().refresh();
            if(bot.deposit().slidesReachedTarget()){
                return false;
            }
            return true;
        };
    }
    public Action setDepositTarget(OrcaV1 bot, double pos ) {
        return telemetryPacket -> {
            //TODO: I dont think you will need to use this for now.
            bot.deposit().setTarget(pos);
            bot.deposit().refresh();
            return false;
        };
    }
    public Action depositSample(OrcaV1 bot) {
        return telemetryPacket -> {
            bot.deposit().refresh();
            if(bot.deposit().getDepositCommand().equals("retract")){
                bot.deposit().setSample();
            }
            if(bot.deposit().slidesReachedTarget()){
                bot.deposit().depositSample();
                sleep(300);
            }
            else{
                return true;
            }
            bot.deposit().retract();
            return false;
        };
    }
    public Action depositSpecimen(OrcaV1 bot ) {
        return telemetryPacket -> {
            bot.deposit().refresh();
            if(bot.deposit().getDepositCommand().equals("retract")){
                bot.deposit().setSpecimen();
            }
            if(bot.deposit().slidesReachedTarget()){
                bot.deposit().scoreSpecimen();
                bot.deposit().refresh();
                if(!bot.deposit().slidesReachedTarget()){
                    return true;
                }
            }
            else{
                return true;
            }
            bot.deposit().retract();
            return false;
        };
    }

    public Action makeArmOuty(OrcaV1 bot){
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("Voltage: "+orca.intake().getRotationVoltage());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
        return telemetryPacket -> {
            if(bot.intake().getIntakeCommand().equals("retract")){
                bot.intake().startIntaking();
                return true;
            }
            if(bot.intake().isArmGoodyOrNottyForAutoonyToDoAutoonyThingies()){
                return true;
            }

            bot.intake().refresh(0,false,false,false,false,false);
            bot.intake().retract();
            return false;
        };
    }

    public Action makeArmInny(OrcaV1 bot){
        return telemetryPacket -> {
                bot.intake().retract();
                return true;
        };
    }

    public Action intakeSampleGround(OrcaV1 bot , double pos ) {
        return telemetryPacket -> {
            if(bot.intake().getIntakeCommand().equals("retract")){
                //TODO: When we implement PID, we can change this
                //bot.intake().setTarget(pos);
                bot.intake().startIntaking();
            }

            bot.intake().refresh(0.75,false,true,false,false,false);
            if(!bot.intake().slidesReachedTarget()&&bot.intake().getCurrentSample().equals("none")){
               return true;
            }
            bot.intake().refresh(0,false,false,false,false,false);
            bot.intake().retract();
            //TODO: REMEMBER YOU NEED TO KEEP REFRESHING THE BOT AFTER THIS ACTION FOR THE TRANSFER TO WORK PROPERLY.
            //This means if you pick up the sample, you next movement should be a parallel action combined with the movement
            //and the orcaRefresh Action with all booleans set to false aside from retract, and slides power set to 0
            return false;
        };
    }

}