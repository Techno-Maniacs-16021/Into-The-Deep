package org.firstinspires.ftc.teamcode.auton.oldPaths;

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
public class OnePlusFour extends LinearOpMode {
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    ElapsedTime timeSinceAttempt = new ElapsedTime();

    double slidePower = 0.0;
    public static double x1;
    public static double x2;
    public static double y1;
    public static double y2;
    public static double h1 = 70;
    public static double h2;


    /*Pose2d sampleNumberOneStart = new Pose2d(-18.05,-28.29, Math.toRadians(-146.9));
    Pose2d sampleNumberOneEnd = new Pose2d(-18.11,-40.59, Math.toRadians(-180));
    Pose2d sampleNumberTwoStart = new Pose2d(-19.5,-28.6,Math.toRadians(-147.9));
    Pose2d sampleNumberTwoEnd = new Pose2d(-21.55,-41.27,Math.toRadians(178));
    Pose2d sampleNumberThree = new Pose2d(-17.6,-46,Math.toRadians(-140));*/

    final boolean onlyPaths = false;

    Pose2d sample1 = new Pose2d(-28,-22.87,Math.toRadians(-122.1));
    Pose2d lineUpSample1 = new Pose2d(-21.9,-48.2,Math.toRadians(156.6));

    Pose2d sample2 = new Pose2d(-15.3,-50,Math.toRadians(-179));

    Pose2d sample3 = new Pose2d(-16.5,-51,Math.toRadians(-159.5));

    Pose2d park = new Pose2d(-54.2,-14.6, Math.toRadians(-90));
    Pose2d submersible = new Pose2d(-54.2,-27, Math.toRadians(90));
    Pose2d pickUpSub = new Pose2d(-54.2,-16, Math.toRadians(90));

    Pose2d lineUpSample = new Pose2d(-13.5,-43.3,Math.toRadians(135));

    Pose2d finalSample = new Pose2d(-8.8,-50,Math.toRadians(135));


    Pose2d interSample =  new Pose2d(-20,-40,Math.toRadians(135));
    Vector2d specimen = new Vector2d(-30.65,-0.45);


    Pose2d pickyUppySample = new Pose2d(-8.8,-50, Math.toRadians(135));

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //orca = initRobot(0,0,0);
        orca = new OrcaV1(hardwareMap,new Pose2d(0,0,0));
        //orcaRefresh(orca,0,false,false,false,false,false);
        while(opModeInInit()&&!opModeIsActive()&&!isStopRequested()){
            telemetry.addLine("press cross for blue alliance, press triangle for red alliance");
            telemetry.addData("currently selected alliance: ", orca.intake().getColorToEject().equals("red") ? "blue" : "red");
            telemetry.update();
            if(gamepad1.cross){
                orca.intake().setColorToEject("red");
            }
            else if(gamepad1.triangle){
                orca.intake().setColorToEject("blue");
            }
            orca.specimenInit();
        }
        waitForStart();



        runningTime.reset();
        timeSinceAttempt.reset();
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
        //Actions.runBlocking(new SequentialAction(
        //        depositSample(orca),parkSlides(orca)
        //));
        //requestOpModeStop();
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(0, 0, 0))
                                .strafeTo(specimen)
                                .build(),
                        setSpecimen(orca)
                ),
                depositSpecimen(orca),
                //sample 1
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(specimen.x,specimen.y,0))
                                .strafeToLinearHeading(sample1.component1(),sample1.heading.toDouble())
                                .build(),
                        retractDeposit(orca)
                ),
                print("reached pos@"+runningTime),
                /*TODO: Parallel intake and transfer while moving*/
                new ParallelAction(
                        orca.actionBuilder(sample1)
                                .setTangent(Math.toRadians(Math.toDegrees(sample1.heading.toDouble()+90)))
                                .splineToSplineHeading(lineUpSample1,Math.toRadians(-90))
                                .build(),
                        new SequentialAction(
                                intakeSampleGround(orca),retractIntake(orca)
                        )
                ),
                //TODO: extend intake while depositing
                new ParallelAction(
                        //setSample(orca),
                        depositSample(orca),
                        orca.actionBuilder(lineUpSample1)
                                .strafeToLinearHeading(finalSample.component1(),finalSample.heading.toDouble())
                                .build(),
                        primeIntake(orca,1.4)
                ),
                //depositSample(orca),
                //sample 2
                new ParallelAction(
                        orca.actionBuilder(finalSample)
                                .strafeToLinearHeading(sample2.component1(),sample2.heading.toDouble())
                                .build(),
                        retractDeposit(orca)
                ),
                print("reached pos@"+runningTime),
                intakeSampleGround(orca),

                new ParallelAction(
                        orca.actionBuilder(sample2)
                                .strafeToLinearHeading(lineUpSample.component1(),lineUpSample.heading.toDouble())
                                .build(),
                        retractIntake(orca)
                ),

                //TODO: extend intake while depositing
                new ParallelAction(
                        //setSample(orca),
                        depositSample(orca),
                        orca.actionBuilder(lineUpSample)
                                .strafeToLinearHeading(finalSample.component1(),finalSample.heading.toDouble())
                                .build(),
                        primeIntake(orca,1.4)
                ),
                //depositSample(orca),

                //sample 3
                new ParallelAction(
                        orca.actionBuilder(finalSample)
                                .strafeToLinearHeading(sample3.component1(),sample3.heading.toDouble())
                                .build(),
                        retractDeposit(orca)
                ),
                print("reached pos@"+runningTime),
                intakeSampleGround(orca),

                new ParallelAction(
                        orca.actionBuilder(sample3)
                                .strafeToLinearHeading(lineUpSample.component1(),lineUpSample.heading.toDouble())
                                .build(),
                        retractIntake(orca)
                ),
                //TODO: extend intake while depositing
                new ParallelAction(
                        //setSample(orca),
                        depositSample(orca),
                        orca.actionBuilder(lineUpSample)
                                .strafeToLinearHeading(finalSample.component1(),finalSample.heading.toDouble())
                                .build(),
                        primeIntake(orca,1)
                ),
                //depositSample(orca),

                //sample 4
                new ParallelAction(

                        orca.actionBuilder(finalSample)
                                .splineToSplineHeading(submersible,Math.toRadians(120))
                                .strafeToConstantHeading(pickUpSub.component1())
                                .build(),
                        retractDeposit(orca)
                ),
                intakeSampleSubmersible(orca),

                new ParallelAction(
                        orca.actionBuilder(pickUpSub)
                                .setTangent(Math.toRadians(-90))
                                .splineToSplineHeading(interSample,Math.toRadians(0))
                                .build(),
                                retractIntake(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(interSample)
                                .setTangent(Math.toRadians(-90))
                                .strafeToConstantHeading(finalSample.component1())
                                .build(),
                        depositSample(orca)
                        ),
                wait(250),
                new ParallelAction(
                        orca.actionBuilder(finalSample)
                                .splineToSplineHeading(park,Math.toRadians(90))
                                .build(),
                        parkSlides(orca)
                ),
                touchBar(orca)

        ));
                /*
                new ParallelAction(
                        orca.actionBuilder(sampleNumberOneStart)
                                .strafeToLinearHeading(sampleNumberOneEnd.component1(),sampleNumberOneEnd.heading.toDouble())
                                .build(),
                        intakeSampleGround(orca),

                new ParallelAction(
                        orca.actionBuilder(sampleNumberOneEnd)
                                .strafeToLinearHeading(lineUpSample.component1(),lineUpSample.heading.toDouble())
                                .build(),
                        retractIntake(orca)
                ),

                setSample(orca),

                orca.actionBuilder(lineUpSample)
                        .strafeTo(finalSample.component1())
                        .build(),

                depositSample(orca),



                new ParallelAction(
                        orca.actionBuilder(finalSample)
                                .strafeToLinearHeading(sampleNumberTwoStart.component1(),sampleNumberTwoStart.heading.toDouble())
                                .build(),
                        retractDeposit(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(sampleNumberTwoStart)
                                .strafeTo(sampleNumberTwoEnd.component1())
                                .build(),
                        intakeSampleGround(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(sampleNumberTwoEnd)
                                .strafeToLinearHeading(lineUpSample.component1(),lineUpSample.heading.toDouble())
                                .build(),
                        retractIntake(orca)
                ),
                setSample(orca),
                orca.actionBuilder(lineUpSample)
                        .strafeTo(finalSample.component1())
                        .build(),

                depositSample(orca),

               // new


                new ParallelAction(
                        orca.actionBuilder(finalSample)
                                .strafeToLinearHeading(sampleNumberThree.component1(),sampleNumberThree.heading.toDouble())
                                .build(),
                        retractDeposit(orca)
                ),
                intakeSampleGround(orca),

                new ParallelAction(
                        orca.actionBuilder(sampleNumberThree)
                                .strafeToLinearHeading(lineUpSample.component1(),lineUpSample.heading.toDouble())
                                .build(),
                        retractIntake(orca)
                ),
                setSample(orca),
                orca.actionBuilder(lineUpSample)
                        .strafeTo(finalSample.component1())
                        .build(),

                depositSample(orca),

                orca.actionBuilder(finalSample)
                        .strafeTo(lineUpSample.component1())
                        .build(),

                retractDeposit(orca)
                )*/



        requestOpModeStop();
    }
    public Action wait(int ms ) {
        return telemetryPacket -> {
            sleep(ms);
            return false;
        };
    }
    public Action print(String str ) {
        return telemetryPacket -> {
            System.out.println(str);
            return false;
        };
    }
    public Action orcaRefresh(OrcaV1 bot, double slidesPower, boolean verticalIntake, boolean angledIntake, boolean reverseIntake, boolean retract, boolean pid ) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            //TODO: Use this anytime the mechanisms are moving while the robot is moving in a parallel action
            bot.deposit().refresh();
            bot.intake().refresh(slidesPower,verticalIntake,angledIntake,reverseIntake,retract,pid);
            return false;
        };
    }
    public Action setSample(OrcaV1 bot ) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
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
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            bot.deposit().setSpecimen();
            bot.deposit().refresh();
            if(bot.deposit().slidesReachedTarget()){
                return false;
            }
            //telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
            //telemetry.addData("target pos: ", bot.deposit().getTarget());
            //telemetry.update();
            //System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
            //System.out.println("target pos: " + bot.deposit().getTarget());
            return true;
        };
    }
    public Action parkSlides(OrcaV1 bot ) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            bot.deposit().refresh();
            bot.deposit().setPark();
            bot.deposit().refresh();

            //System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
            //System.out.println("target pos: " + bot.deposit().getTarget());

            return !bot.deposit().slidesReachedTarget();
        };
    }
    public Action touchBar(OrcaV1 bot ) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            bot.deposit().refresh();
            bot.deposit().park();
            bot.deposit().refresh();

            sleep(500);
            return false;
        };
    }
    public Action retractDeposit(OrcaV1 bot ) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            if(!bot.deposit().getDepositCommand().equals("retract")){
                bot.deposit().retract();
            }
            bot.deposit().refresh();
            if(!bot.deposit().slidesReachedTarget()){
                return true;
            }
            //telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
            //telemetry.addData("target pos: ", bot.deposit().getTarget());
            //telemetry.update();
            //System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
            //System.out.println("target pos: " + bot.deposit().getTarget());
            System.out.println("retracted@"+runningTime);
            return false;
        };
    }
    public Action setDepositTarget(OrcaV1 bot, double pos ) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            //TODO: I dont think you will need to use this for now.
            bot.deposit().setTarget(pos);
            bot.deposit().refresh();
            return false;
        };
    }
    public Action depositSample(OrcaV1 bot) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            bot.deposit().refresh();
            if(bot.deposit().getDepositCommand().equals("retract")){
                bot.deposit().setSample();
            }
            bot.deposit().refresh();
            if(bot.deposit().getCurrentSlidePosition()>2.4){
                bot.deposit().depositSample();
                bot.deposit().refresh();
            }
            if(bot.deposit().slidesReachedTarget()){
                bot.deposit().depositSample();
                bot.deposit().refresh();
                sleep(150);
            }
            else{
                //telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
                //telemetry.addData("target pos: ", bot.deposit().getTarget());
                //telemetry.update();
                return true;
            }
            //bot.deposit().retract();
            return false;
        };
    }
    public Action depositSpecimen(OrcaV1 bot ) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
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
                //telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
                //telemetry.addData("target pos: ", bot.deposit().getTarget());
                //telemetry.update();
                //System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
                //System.out.println("target pos: " + bot.deposit().getTarget());
                return true;
            }
            return false;
        };
    }
    public Action primeIntake(OrcaV1 bot, double pos ) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            bot.intake().startIntaking();
            bot.intake().setTarget(pos);
            bot.intake().refresh(0,false,false,false,false,true);

            return !bot.intake().slidesReachedTarget();
        };
    }
    public Action intakeSampleGround(OrcaV1 bot) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            if(!bot.intake().getIntakeCommand().equals("intake")){
                //TODO: When we implement PID, we can change this
                //bot.intake().setTarget(pos);
                bot.intake().startIntaking();
                System.out.println("intaking@"+runningTime);
            }
            bot.intake().angledIntake();
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
    public Action intakeSampleSubmersible(OrcaV1 bot) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            if(!bot.intake().getIntakeCommand().equals("intake")){
                //TODO: When we implement PID, we can change this
                //bot.intake().setTarget(pos);
                bot.intake().startIntaking();
                System.out.println("intaking@"+runningTime);
            }
            if(timeSinceAttempt.milliseconds()<1250&&timeSinceAttempt.milliseconds()>1050){
                bot.intake().refresh(0.75,false,false,false,false,false);
            }
            else if (timeSinceAttempt.milliseconds()>1250){
                timeSinceAttempt.reset();
                bot.intake().refresh(0,true,false,false,false,false);
            }
            else{
                bot.intake().refresh(0,true,false,false,false,false);
            }

            if((!bot.intake().slidesReachedTarget()&&bot.intake().getCurrentSample().equals("none"))){
                return true;
            }
            //bot.intake().retract();
            //bot.intake().refresh(0,false,false,false,true,false);
            //TODO: REMEMBER YOU NEED TO KEEP REFRESHING THE BOT AFTER THIS ACTION FOR THE TRANSFER TO WORK PROPERLY.
            //This means if you pick up the sample, you next movement should be a parallel action combined with the movement
            //and the orcaRefresh Action with all booleans set to false aside from retract, and slides power set to 0
            if(!orca.intake().getCurrentColorReading().equals(orca.intake().getColorToEject())){
                for(int k = 0; k<32&&!orca.intake().getCurrentColorReading().equals("none") ; k++){
                    bot.intake().refresh(0,true,false,false,false,false);
                    sleep(25);
                }
                if(orca.intake().getCurrentColorReading().equals("none")){
                    return true;
                }
            }
            return orca.intake().getCurrentColorReading().equals(orca.intake().getColorToEject());
        };
    }
    public Action retractIntake(OrcaV1 bot) {
        if(onlyPaths){
            return telemetryPacket -> {
                return false;
            };
        }
        return telemetryPacket -> {
            bot.intake().refresh(0,false,false,false,true,false);
            //System.out.println(bot.intake().getIntakeCommand() + runningTime);
            if(bot.intake().getIntakeCommand().equals("standby")){
                //System.out.println("rr: " + bot.intake().getRotationVoltage());
                if((Math.abs(1.4-bot.intake().getRotationVoltage()) < 0.04)){
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    bot.intake().refresh(0,false,false,false,true,false);
                    //System.out.println("rr: " + bot.intake().getRotationVoltage());
                    return !(Math.abs(1.4-bot.intake().getRotationVoltage()) < 0.03);
                }
                return true;
            }

            if(!bot.intake().getIntakeCommand().equals("retract")){
                //TODO: When we implement PID, we can change this
                bot.intake().retract();
            }

            return true;
        };
    }

}