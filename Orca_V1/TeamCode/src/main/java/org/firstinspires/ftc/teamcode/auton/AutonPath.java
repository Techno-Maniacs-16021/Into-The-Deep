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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.tuning.localization.Drawing;
import org.firstinspires.ftc.teamcode.robots.OrcaV1;

@Autonomous
@Config
public class AutonPath extends LinearOpMode {
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    double slidePower = 0.0;
    public static double x1;
    public static double x2;
    public static double y1;
    public static double y2;
    public static double h1 = 70;
    public static double h2;

    Pose2d preSample = new Pose2d(-13.5,-43.3,Math.toRadians(135));

    Pose2d sampleNumberOneStart = new Pose2d(-18.05,-28.29, Math.toRadians(-146.9));
    Pose2d sampleNumberOneEnd = new Pose2d(-18.11,-40.59, Math.toRadians(-180));
    Pose2d sampleNumberTwoStart = new Pose2d(-19.5,-28.6,Math.toRadians(-147.9));
    Pose2d sampleNumberTwoEnd = new Pose2d(-21.55,-41.27,Math.toRadians(178));
    Pose2d sampleNumberThree = new Pose2d(-17.6,-46,Math.toRadians(-140));
    Pose2d finalSample = new Pose2d(-8.8,-50,Math.toRadians(135));
    Vector2d specimen = new Vector2d(-30.65,-1.45);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //orca = initRobot(0,0,0);
        orca = new OrcaV1(hardwareMap,new Pose2d(0,0,0));
        //orcaRefresh(orca,0,false,false,false,false,false);
        //orca.init();
        waitForStart();



        runningTime.reset();
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("Voltage: "+orca.intake().getRotationVoltage());
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), orca.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
        /*Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(0, 0, 0))
                                .strafeTo(preSpecimen)
                                .build(),
                        intakeSampleGround(orca,2.5)//setSpecimen(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(preSpecimen.x,preSpecimen.y,0))
                                .strafeTo(finalSpecimen)
                                .build(),
                        retractIntake(orca,00)
                ),
                wait(300),
                depositSample(orca)
        ));*/
        Actions.runBlocking(new SequentialAction(

                ));
        requestOpModeStop();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(0, 0, 0))
                                .strafeTo(specimen)
                                .build()//,
                        //setSpecimen(orca)
                ),
                //depositSpecimen(orca),
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(specimen.x,specimen.y,0))
                                .strafeToLinearHeading(sampleNumberOneStart.component1(),sampleNumberOneStart.heading.toDouble())
                                .build()//,
                        //retractDeposit(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(sampleNumberOneStart)
                                .strafeToLinearHeading(sampleNumberOneEnd.component1(),sampleNumberOneEnd.heading.toDouble())
                                .build()//,
                        //intakeSampleGround(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(sampleNumberOneEnd)
                                .strafeToLinearHeading(preSample.component1(),preSample.heading.toDouble())
                                .build()//,
                        //retractIntake(orca)
                ),

                //setSample(orca),

                orca.actionBuilder(preSample)
                        .strafeTo(finalSample.component1())
                        .build(),

                //depositSample(orca),



                new ParallelAction(
                        orca.actionBuilder(finalSample)
                                .strafeToLinearHeading(sampleNumberTwoStart.component1(),sampleNumberTwoStart.heading.toDouble())
                                .build()//,
                        //retractDeposit(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(sampleNumberTwoStart)
                                .strafeTo(sampleNumberTwoEnd.component1())
                                .build()//,
                        //intakeSampleGround(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(sampleNumberTwoEnd)
                                .strafeToLinearHeading(preSample.component1(),preSample.heading.toDouble())
                                .build()//,
                        //retractIntake(orca)
                ),
                //setSample(orca),
                orca.actionBuilder(preSample)
                        .strafeTo(finalSample.component1())
                        .build(),

                //depositSample(orca),

                // new


                new ParallelAction(
                        orca.actionBuilder(finalSample)
                                .strafeToLinearHeading(sampleNumberThree.component1(),sampleNumberThree.heading.toDouble())
                                .build()//,
                        //retractDeposit(orca)
                ),
                //intakeSampleGround(orca),

                new ParallelAction(
                        orca.actionBuilder(sampleNumberThree)
                                .strafeToLinearHeading(preSample.component1(),preSample.heading.toDouble())
                                .build()//,
                        //retractIntake(orca)
                ),
                //setSample(orca),
                orca.actionBuilder(preSample)
                        .strafeTo(finalSample.component1())
                        .build(),

                //depositSample(orca),

                orca.actionBuilder(finalSample)
                        .strafeTo(preSample.component1())
                        .build()//,

                //retractDeposit(orca)


        ));



        requestOpModeStop();
    }
    public Action wait(int ms ) {
        return telemetryPacket -> {
            //TODO: Use this anytime the mechanisms are moving while the robot is moving in a parallel action
            sleep(ms);
            return false;
        };
    }
    public Action orcaRefresh(OrcaV1 bot, double slidesPower, boolean verticalIntake, boolean angledIntake, boolean reverseIntake, boolean retract, boolean pid, boolean moving ) {
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
            if(!bot.deposit().slidesReachedTarget()){
                return true;
            }
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
            telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
            telemetry.addData("target pos: ", bot.deposit().getTarget());
            telemetry.update();
            System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
            System.out.println("target pos: " + bot.deposit().getTarget());
            return true;
        };
    }
    public Action retractDeposit(OrcaV1 bot ) {
        return telemetryPacket -> {
            if(!bot.deposit().getDepositCommand().equals("retract")){
                bot.deposit().retract();
            }
            bot.deposit().refresh();
            if(!bot.deposit().slidesReachedTarget()){
                return true;
            }
            telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
            telemetry.addData("target pos: ", bot.deposit().getTarget());
            telemetry.update();
            System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
            System.out.println("target pos: " + bot.deposit().getTarget());
            return false;
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
            bot.deposit().refresh();
            if(bot.deposit().slidesReachedTarget()){
                bot.deposit().depositSample();
                bot.deposit().refresh();
                sleep(600);
            }
            else{
                telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
                telemetry.addData("target pos: ", bot.deposit().getTarget());
                telemetry.update();
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
                telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
                telemetry.addData("target pos: ", bot.deposit().getTarget());
                telemetry.update();
                System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
                System.out.println("target pos: " + bot.deposit().getTarget());
                return true;
            }
            return false;
        };
    }

    public Action intakeSampleGround(OrcaV1 bot) {
        return telemetryPacket -> {
            if(!bot.intake().getIntakeCommand().equals("intake")){
                //TODO: When we implement PID, we can change this
                //bot.intake().setTarget(pos);
                bot.intake().startIntaking();
                bot.intake().angledIntake();
            }

            bot.intake().refresh(0.75,false,true,false,false,false);
            if(!bot.intake().slidesReachedTarget()&&bot.intake().getCurrentSample().equals("none")){
                return true;
            }
            //bot.intake().retract();
            //bot.intake().refresh(0,false,false,false,true,false);
            //TODO: REMEMBER YOU NEED TO KEEP REFRESHING THE BOT AFTER THIS ACTION FOR THE TRANSFER TO WORK PROPERLY.
            //This means if you pick up the sample, you next movement should be a parallel action combined with the movement
            //and the orcaRefresh Action with all booleans set to false aside from retract, and slides power set to 0
            return false;
        };
    }
    public Action retractIntake(OrcaV1 bot) {
        return telemetryPacket -> {
            bot.intake().refresh(0,false,false,false,true,false);
            System.out.println(bot.intake().getIntakeCommand());
            if(bot.intake().getIntakeCommand().equals("standby")){
                System.out.println("rr: " + bot.intake().getRotationVoltage());
                return !(Math.abs(1.4-bot.intake().getRotationVoltage()) < 0.01);
            }

            if(!bot.intake().getIntakeCommand().equals("retract")){
                //TODO: When we implement PID, we can change this
                bot.intake().retract();
            }

            return true;
        };
    }

}