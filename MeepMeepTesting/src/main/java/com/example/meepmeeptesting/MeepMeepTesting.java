package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Actions;
public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8.3, 66, Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(-8.3, 35), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(-8.3, 38.5), Math.toRadians(110))
                .strafeToLinearHeading(new Vector2d(-36, 38.5), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-36, 15), (Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-45, 15), (Math.toRadians(-90)))
                .strafeToLinearHeading(new Vector2d(-48, 56), (Math.toRadians(-90)))

                .strafeToLinearHeading(new Vector2d(-37, 56), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-37, 62), Math.toRadians(-90))

//                .strafeToLinearHeading(new Vector2d(-37, 56), Math.toRadians(-90))
//                .strafeToSplineHeading(new Vector2d(-5, 41), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(-5, 35), Math.toRadians(90))




                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}