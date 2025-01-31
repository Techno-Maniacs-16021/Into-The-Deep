package org.firstinspires.ftc.teamcode.auton.oldPaths;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.OrcaV1;

public abstract class AutonActions  extends LinearOpMode {
    public static OrcaV1 initRobot(HardwareMap hardwareMap, double x, double y, double heading){
        OrcaV1 orcaV1 = new OrcaV1(hardwareMap,new Pose2d(x,y,heading));
        //orcaV1.init();
        return orcaV1;
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
    public Action touchBar(OrcaV1 bot ) {
        return telemetryPacket -> {
            bot.deposit().refresh();
            bot.deposit().park();
            bot.deposit().refresh();

            sleep(500);
            return false;
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
