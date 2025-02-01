package com.example.meepmeeptestingb;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {

        Pose2d start = new Pose2d(-65.5,-11,Math.toRadians(0));

        Pose2d pickUp1 = new Pose2d(-43.4,-34,Math.toRadians(-45.5));
        Pose2d pickUp2 = new Pose2d(-41.5,-42.7,Math.toRadians(-47.2));
        Pose2d pickUp3 = new Pose2d(-26.9,-42.3,Math.toRadians(-89.6));

        Pose2d dropOff1 = new Pose2d(-43.4,-34,Math.toRadians(-130.3));
        Pose2d dropOff2 = new Pose2d(-41.5,-42.7,Math.toRadians(-125));
        Pose2d dropOff3 = new Pose2d(-60,-42.3,Math.toRadians(-89.6));

        Vector2d specimenDrop = new Vector2d(-34.85,-11);
        Vector2d collectSpecimen = new Vector2d(-63.8,-42.3);


        //transformations
        start = new Pose2d(-start.component1().y, start.component1().x,start.heading.toDouble()+Math.toRadians(90));

        pickUp1 = new Pose2d(-pickUp1.component1().y, pickUp1.component1().x,pickUp1.heading.toDouble()+Math.toRadians(90));
        pickUp2 = new Pose2d(-pickUp2.component1().y, pickUp2.component1().x,pickUp2.heading.toDouble()+Math.toRadians(90));
        pickUp3 = new Pose2d(-pickUp3.component1().y, pickUp3.component1().x,pickUp3.heading.toDouble()+Math.toRadians(90));

        dropOff1 = new Pose2d(-dropOff1.component1().y, dropOff1.component1().x,dropOff1.heading.toDouble()+Math.toRadians(90));
        dropOff2 = new Pose2d(-dropOff2.component1().y, dropOff2.component1().x,dropOff2.heading.toDouble()+Math.toRadians(90));
        dropOff3 = new Pose2d(-dropOff3.component1().y, dropOff3.component1().x,dropOff3.heading.toDouble()+Math.toRadians(90));

        specimenDrop = new Vector2d(-specimenDrop.y, specimenDrop.x);
        collectSpecimen = new Vector2d(-collectSpecimen.y, collectSpecimen.x);


        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13,13)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(start)
                .strafeTo(specimenDrop)
                .strafeToLinearHeading(pickUp1.component1(),pickUp1.heading)
                .turn(dropOff1.heading.toDouble()-pickUp1.heading.toDouble())
                .strafeToLinearHeading(pickUp2.component1(),pickUp2.heading)
                .turn(dropOff2.heading.toDouble()-pickUp2.heading.toDouble())
                .strafeToLinearHeading(pickUp3.component1(),pickUp3.heading)
                .strafeTo(dropOff3.component1())
                .turn(Math.toRadians(90)-dropOff3.heading.toDouble())
                .strafeTo(collectSpecimen)
                .strafeTo(specimenDrop)
                .strafeTo(collectSpecimen)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}