package com.example.meepmeeptestingb;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {

        Pose2d preSpecimen = new Pose2d(5.7,32,Math.toRadians(90));
        Pose2d finalSpecimen = new Pose2d(5.7,30,Math.toRadians(90));

        Pose2d goUpPlace = new Pose2d(52,52,Math.toRadians(-135));
        Pose2d depositPlace = new Pose2d(56,56,Math.toRadians(-135));

        Pose2d pickupOneStart = new Pose2d(48,44, Math.toRadians(-90));
        Pose2d pickupTwoStart = new Pose2d(56,44, Math.toRadians(-90));
        Pose2d pickupThreeStart = new Pose2d(48,24, Math.toRadians(0));

        Pose2d pickupOneEnd = new Pose2d(48,34, Math.toRadians(-90));
        Pose2d pickupTwoEnd = new Pose2d(56,34, Math.toRadians(-90));
        Pose2d pickupThreeEnd = new Pose2d(58,24, Math.toRadians(0));

        Pose2d parkingPlace = new Pose2d(20,-16,Math.toRadians(0));

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-27.9,-1.7,0))
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(-14.4,-49.5,Math.toRadians(160)),Math.toRadians(-20))
                        .strafeTo(new Vector2d(-7.3,-52.2))
                        .strafeTo(new Vector2d(-14.4,-49.5))
                        .build()
        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(6, 70, Math.toRadians(90)))
                    // Does all the tingies for placing down your facy thingie
                    .strafeTo(preSpecimen.component1())
                        // Make arm go down and slides go up
                    .strafeTo(finalSpecimen.component1())
                    .strafeTo(finalSpecimen.component1()) // So memap can see it
                        // Make the fancy thingie be placed

                    // Goes to pick up your less fancy vanilla bricks
                    .setTangent(Math.toRadians(90))
                    .splineTo(pickupOneStart.component1(),Math.toRadians(-90))
                    .strafeTo(pickupOneStart.component1()) // So memap can see it
                        // Start intaking

                    .strafeTo(pickupOneEnd.component1())
                    .strafeTo(pickupOneEnd.component1())// So memap can see it
                        // Hopefully done with intaking

                    // Goes to drop off your less fancy bricks
                    .strafeToLinearHeading(goUpPlace.component1(),Math.toRadians(-135))
                    .strafeTo(goUpPlace.component1()) // So memap can see it
                        // Slides up!
                    .strafeTo(depositPlace.component1())
                    .strafeTo(depositPlace.component1()) // So memap can see it
                        // Dump!
                    .strafeTo(goUpPlace.component1())
                    .strafeTo(goUpPlace.component1()) // So memap can see it
                        // Slides down


                    // Goes to pick up your less fancy vanilla bricks again
                    .setTangent(Math.toRadians(45+180))
                    .splineTo(pickupTwoStart.component1(),Math.toRadians(-90))
                    .strafeTo(pickupTwoStart.component1()) // So memap can see it
                        // Start intaking
                    .strafeTo(pickupTwoEnd.component1())
                    .strafeTo(pickupTwoEnd.component1())// So memap can see it
                        // Hopefully done with intaking


                    // Goes to drop off your less fancy bricks again
                    .strafeToLinearHeading(goUpPlace.component1(),Math.toRadians(-135))
                    .strafeTo(goUpPlace.component1()) // So memap can see it
                        // Slides up!
                    .strafeTo(depositPlace.component1())
                    .strafeTo(depositPlace.component1()) // So memap can see it
                        // Dump!
                    .strafeTo(goUpPlace.component1())
                    .strafeTo(goUpPlace.component1()) // So memap can see it
                        // Slides down


                    // Goes to pick up your less fancy vanilla bricks but with a twise (get it? a twist?!)
                    .setTangent(Math.toRadians(45+180))

                    .splineTo(pickupThreeStart.component1(),Math.toRadians(0))
                    .strafeTo(pickupThreeStart.component1()) // So memap can see it
                        // Start intaking
                    .strafeTo(pickupThreeEnd.component1())
                    .strafeTo(pickupThreeEnd.component1())// So memap can see it
                        // Hopefully done with intaking


                    // Goes to drop off your less fancy bricks
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(goUpPlace,Math.toRadians(45))
                    .strafeTo(goUpPlace.component1()) // So memap can see it
                        // Slides up!
                    .strafeTo(depositPlace.component1())
                    .strafeTo(depositPlace.component1()) // So memap can see it
                        // Dump!
                    .strafeTo(goUpPlace.component1())
                    .strafeTo(goUpPlace.component1()) // So memap can see it
                        // Slides down


                // Goes to part the big pretty robot
                    .setTangent(Math.toRadians(45+180))
                    .splineToLinearHeading(parkingPlace,Math.toRadians(180))
                    .build()*/
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}