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
public class FivePlusZero extends LinearOpMode {
    OrcaV1 orca;
    ElapsedTime runningTime = new ElapsedTime();
    ElapsedTime waitingTime = new ElapsedTime();
    boolean waiting = false;

    double slidePower = 0.0;
    public static double x1;
    public static double x2;
    public static double y1;
    public static double y2;
    public static double h1 = 70;
    public static double h2;

    Pose2d pickUp1 = new Pose2d(-22.1,27,Math.toRadians(134.5));
    Pose2d pickUp2 = new Pose2d(-24,35.7,Math.toRadians(132.8));

    Pose2d pickUp3 = new Pose2d(-38.6,35.3,Math.toRadians(90.4));

    Pose2d dropOff1 = new Pose2d(-22.1,27,Math.toRadians(50.7));

    Pose2d dropOff2 = new Pose2d(-24,35.7,Math.toRadians(55));
    Pose2d dropOff3 = new Pose2d(-5.5,35.3,Math.toRadians(90.4));
    int waitReady = 800;
    int waitPickup = 200;

    /* strafing stuff
    Pose2d pickUp1 = new Pose2d(-49,41.7,Math.toRadians(180));
    Vector2d pickUp2 = new Vector2d(-49,52);

    Vector2d pickUp3 = new Vector2d(-49,58.5);

    Vector2d dropOff1 = new Vector2d(-14,41.7);

    Vector2d dropOff2 = new Vector2d(-14,52);
    Vector2d dropOff3 = new Vector2d(-14,57.5);*/

    Vector2d specimenDrop = new Vector2d(-30.65,4);
    Vector2d collectSpecimen = new Vector2d(-1.7,35.3);



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
        waitingTime.reset();
        TelemetryPacket packet = new TelemetryPacket();
        packet.addLine("Voltage: "+orca.intake().getRotationVoltage());
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), orca.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(0, 0, 0))
                                .strafeTo(specimenDrop)
                                .build(),
                        new SequentialAction(setSpecimen(orca),smartWait(100),depositSpecimen(orca))
                ),
                /*
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(specimenDrop.x,specimenDrop.y,Math.toRadians(0)))
                            .setTangent(Math.toRadians(30))
                            .splineToSplineHeading(pickUp1,Math.toRadians(-45))
                            .build(),
                        retractDeposit(orca)
                ),
                orca.actionBuilder(pickUp1)
                        .strafeTo(dropOff1)
                        .strafeTo(pickUp1.component1())
                        .strafeTo(pickUp2)
                        .strafeTo(dropOff2)
                        .strafeTo(pickUp2)
                        .strafeTo(pickUp3)
                        .strafeTo(dropOff3)
                        .build(),

                 */

                // USING INTAKES AND OUTTAKES - 17s
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(specimenDrop.x,specimenDrop.y,Math.toRadians(0)))
                                .strafeToLinearHeading(pickUp1.component1(),pickUp1.heading)
                                .build(),
                        intakeSampleGround(orca,0.5),
                        retractDeposit(orca)
                ),
                new ParallelAction(
                        orca.actionBuilder(pickUp1)
                                .turn(dropOff1.heading.toDouble()-pickUp1.heading.toDouble())
                                .build(),
                        new SequentialAction(smartWait(900), outtakeSampleGround(orca))
                ),
                new ParallelAction(
                        orca.actionBuilder(dropOff1)
                                .strafeToLinearHeading(pickUp2.component1(),pickUp2.heading)
                                .build(),
                        setIntake(orca,0.5)
                ),

                intakeSampleGround(orca,1),
                new ParallelAction(
                        orca.actionBuilder(pickUp2)
                                .turn(dropOff2.heading.toDouble()-pickUp2.heading.toDouble())
                                .build(),
                        new SequentialAction(smartWait(900),outtakeSampleGround(orca))
                ),
                new ParallelAction(
                        orca.actionBuilder(dropOff2)
                                .strafeToLinearHeading(pickUp3.component1(),pickUp3.heading)
                                .build(),
                        setIntake(orca,1)
                ),
                intakeSampleGround(orca,1),

                new ParallelAction(
                        orca.actionBuilder(pickUp3)
                                .strafeTo(dropOff3.component1())
                                .turn(Math.toRadians(180)-dropOff3.heading.toDouble())
                                .strafeTo(collectSpecimen)
                                .build(),
                        new SequentialAction(
                                setIntake(orca, 0.5),
                                smartWait(250),
                                outtakeSampleGround(orca),
                                retractIntake(orca),
                                readyPickUpSpecimen(orca),
                                smartWait(waitReady+400),
                                pickUpSpecimen(orca),
                                smartWait(waitPickup)
                        )
                ),

                // specimen scoring

                /*new ParallelAction(
                        orca.actionBuilder(dropOff3)
                                .strafeToLinearHeading(collectSpecimen,Math.toRadians(180))
                                .build(),
                        new SequentialAction(
                                retractIntake(orca),
                                readyPickUpSpecimen(orca)
                        )
                ),*/
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(collectSpecimen.x,collectSpecimen.y,Math.toRadians(180)))
                                .strafeToLinearHeading(new Vector2d(specimenDrop.x, specimenDrop.y-12),Math.toRadians(0))
                                .build(),
                        new SequentialAction(setSpecimen(orca),smartWait(600),depositSpecimen(orca))
                ),

                //SPECIMEN 3

                new ParallelAction(
                        orca.actionBuilder(new Pose2d(specimenDrop.x, specimenDrop.y-12, Math.toRadians(0)))
                                .strafeToLinearHeading(collectSpecimen,Math.toRadians(180))
                                .build(),
                        new SequentialAction(
                                retractDeposit(orca),
                                readyPickUpSpecimen(orca),
                                smartWait(waitReady),
                                pickUpSpecimen(orca),
                                smartWait(waitPickup)
                                )
                ),
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(collectSpecimen.x,collectSpecimen.y,Math.toRadians(180)))
                                .strafeToLinearHeading(new Vector2d(specimenDrop.x, specimenDrop.y-9),Math.toRadians(0))
                                .build(),
                        new SequentialAction(setSpecimen(orca),smartWait(700),depositSpecimen(orca))
                ),

                //SPECIMEN 4

                new ParallelAction(
                        orca.actionBuilder(new Pose2d(specimenDrop.x, specimenDrop.y-9, Math.toRadians(0)))
                                .strafeToLinearHeading(collectSpecimen,Math.toRadians(180))
                                .build(),
                        new SequentialAction(
                                retractDeposit(orca),
                                readyPickUpSpecimen(orca),
                                smartWait(waitReady),
                                pickUpSpecimen(orca),
                                smartWait(waitPickup)
                                )
                ),
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(collectSpecimen.x,collectSpecimen.y,Math.toRadians(180)))
                                .strafeToLinearHeading(new Vector2d(specimenDrop.x, specimenDrop.y-6),Math.toRadians(0))
                                .build(),
                        new SequentialAction(setSpecimen(orca),smartWait(800),depositSpecimen(orca))
                ),

                //SPECIMEN 5

                new ParallelAction(
                        orca.actionBuilder(new Pose2d(specimenDrop.x, specimenDrop.y-6, Math.toRadians(0)))
                                .strafeToLinearHeading(collectSpecimen,Math.toRadians(180))
                                .build(),
                        new SequentialAction(
                                retractDeposit(orca),
                                readyPickUpSpecimen(orca),
                                smartWait(waitReady),
                                pickUpSpecimen(orca),
                                smartWait(waitPickup)
                                )
                ),
                new ParallelAction(
                        orca.actionBuilder(new Pose2d(collectSpecimen.x,collectSpecimen.y,Math.toRadians(180)))
                                .strafeToLinearHeading(new Vector2d(specimenDrop.x, specimenDrop.y-3),Math.toRadians(0))
                                .build(),
                        new SequentialAction(setSpecimen(orca),smartWait(1000),depositSpecimen(orca))
                )

        ));


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
            //telemetry.addData("current pos: ",bot.deposit().getCurrentSlidePosition());
            //telemetry.addData("target pos: ", bot.deposit().getTarget());
            //telemetry.update();
            //System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
            //System.out.println("target pos: " + bot.deposit().getTarget());
            return true;
        };
    }
    public Action parkSlides(OrcaV1 bot ) {
        return telemetryPacket -> {
            bot.deposit().refresh();
            bot.deposit().setPark();
            bot.deposit().refresh();

            //System.out.println("current pos: " + bot.deposit().getCurrentSlidePosition());
            //System.out.println("target pos: " + bot.deposit().getTarget());

            return !bot.deposit().slidesReachedTarget();
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
        return telemetryPacket -> {
            //TODO: I dont think you will need to use this for now.
            bot.deposit().setTarget(pos);
            bot.deposit().refresh();
            return false;
        };
    }
    public Action depositSpecimen(OrcaV1 bot ) {
        return telemetryPacket -> {
            bot.deposit().refresh();
            if(bot.deposit().getCurrentSlidePosition()>1.3){
                bot.deposit().scoreSpecimen();
                bot.deposit().refresh();
                return true;
            }
            else{
                return false;
            }
        };
    }
    public Action setIntake(OrcaV1 bot, double pos ) {
        return telemetryPacket -> {
            bot.intake().setTarget(pos);
            bot.intake().refresh(0,false,false,false,false,true);
            return !bot.intake().slidesReachedTarget();
        };
    }
    public Action intakeSampleGround(OrcaV1 bot, double intakeSlidePwr) {
        return telemetryPacket -> {
            if(!bot.intake().getIntakeCommand().equals("intake")){
                //TODO: When we implement PID, we can change this
                //bot.intake().setTarget(pos);
                bot.intake().startIntaking();
                bot.intake().angledIntake();
                System.out.println("intaking@"+runningTime);
            }
            bot.intake().angledIntake();

            bot.intake().refresh(intakeSlidePwr,false,true,false,false,false);
            if(bot.intake().getCurrentColorReading().equals("blue")||bot.intake().getCurrentColorReading().equals("red")){
                bot.intake().verticalIntake();
                for(int k = 0; k < 8; k++){
                    bot.intake().refresh(0,true,false,false,false,false);
                }
                for(int k = 0; k < 8; k++){
                    bot.intake().refresh(0,false,false,false,false,false);
                }
                return !bot.intake().slidesReachedTarget()&&bot.intake().getCurrentSample().equals("none");
            }
            else{
                return true;
            }
            //bot.intake().retract();
            //bot.intake().refresh(0,false,false,false,true,false);
            //TODO: REMEMBER YOU NEED TO KEEP REFRESHING THE BOT AFTER THIS ACTION FOR THE TRANSFER TO WORK PROPERLY.
            //This means if you pick up the sample, you next movement should be a parallel action combined with the movement
            //and the orcaRefresh Action with all booleans set to false aside from retract, and slides power set to 0
        };
    }
    public Action readyPickUpSpecimen(OrcaV1 bot) {
        return telemetryPacket -> {
            bot.deposit().specimenIntake();
            bot.deposit().refresh();
            return false;
        };
    }
    public Action pickUpSpecimen(OrcaV1 bot) {
        return telemetryPacket -> {
            bot.deposit().closeClaw();
            bot.deposit().refresh();
            return false;
        };
    }
    public Action smartWait(double timeMS) {
        return telemetryPacket -> {
            if(!waiting){
                waitingTime.reset();
                waiting = true;
            }
            if(waitingTime.milliseconds()<timeMS){
                return true;
            }
            waiting = false;
            return false;
        };
    }
    public Action outtakeSampleGround(OrcaV1 bot) {
        return telemetryPacket -> {
            for(int k = 0 ; k < 50; k++){
                orca.intake().refresh(0,false,false,true,false,false);
            }
            return false;
        };
    }
    public Action retractIntake(OrcaV1 bot) {
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