package com.example.meepmeeptestingb;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {

        Pose2d pickUp1 = new Pose2d(34,-43.4,Math.toRadians(44.5));
        Pose2d pickUp2 = new Pose2d(42.7,-41.5,Math.toRadians(42.8));

        Pose2d pickUp3 = new Pose2d(42.3,-26.9,Math.toRadians(0.4));

        Pose2d dropOff1 = new Pose2d(34,-43.4,Math.toRadians(-40.3));

        Pose2d dropOff2 = new Pose2d(42.7,-41.5,Math.toRadians(-35));
        Pose2d dropOff3 = new Pose2d(42.3,-60,Math.toRadians(0.4));

        Vector2d specimenDrop = new Vector2d(11,-34.85);
        Vector2d collectSpecimen = new Vector2d(42.3,-63.8);

        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13,13)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11, -65.5, Math.toRadians(-90)))
                .strafeTo(specimenDrop)
                .strafeToLinearHeading(pickUp1.component1(),pickUp1.heading)
                .turn(dropOff1.heading.toDouble()-pickUp1.heading.toDouble())
                .strafeToLinearHeading(pickUp2.component1(),pickUp2.heading)
                .turn(dropOff2.heading.toDouble()-pickUp2.heading.toDouble())
                .strafeToLinearHeading(pickUp3.component1(),pickUp3.heading)
                .strafeTo(dropOff3.component1())
                .turn(Math.toRadians(90)-dropOff3.heading.toDouble())
                .strafeTo(collectSpecimen)
                .strafeToLinearHeading(new Vector2d(specimenDrop.x+12, specimenDrop.y),Math.toRadians(-90))
                .strafeToLinearHeading(collectSpecimen,Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(specimenDrop.x+9, specimenDrop.y),Math.toRadians(-90))
                .strafeToLinearHeading(collectSpecimen,Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(specimenDrop.x+6, specimenDrop.y),Math.toRadians(-90))
                .strafeToLinearHeading(collectSpecimen,Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(specimenDrop.x+3, specimenDrop.y),Math.toRadians(-90))
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}