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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.62795)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(40, 63.5, Math.toRadians(-90)))


                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)))

                .strafeToLinearHeading(new Vector2d(49, 49), (Math.toRadians(-88)))
                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(56, 50), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(57, 57), (Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(43, 27.5), (Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(47.5, 27.5), (Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(43, 25), (Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(57, 58), (Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(35,12), (Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(20,12), (Math.toRadians(0)))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}