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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.71)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38, 52, Math.toRadians(90)))


                .strafeToLinearHeading(new Vector2d(-15, 15), (Math.toRadians(135)))

//                .strafeToLinearHeading(new Vector2d(50, 49), (Math.toRadians(-90)))
//                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)))
//                .strafeToLinearHeading(new Vector2d(58, 48), Math.toRadians(-86))
//                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)))
//                .strafeToLinearHeading(new Vector2d(60, 47), (Math.toRadians(-63)))
//                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)))
////                .strafeToSplineHeading(new Vector2d(40,20), (Math.toRadians(-160)))
//                .strafeToSplineHeading(new Vector2d(30,12), (Math.toRadians(-180)))
//                .strafeToLinearHeading(new Vector2d(58, 58), (Math.toRadians(-135)))
//                .strafeToSplineHeading(new Vector2d(25,8), (Math.toRadians(0)))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}