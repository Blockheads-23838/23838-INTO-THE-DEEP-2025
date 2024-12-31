package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(150, 140, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .strafeTo(new Vector2d(61, 0)) //preload

                .strafeTo(new Vector2d(45, 0)) //pushticks
                .splineToSplineHeading(new Pose2d(102, -57, Math.toRadians(180)), Math.toRadians(0))
                .setReversed(true)
                .strafeTo(new Vector2d(105, -71))
                .strafeTo(new Vector2d(30, -71))
                .strafeTo(new Vector2d(105, -71))
                .strafeTo(new Vector2d(105, -90))
                .strafeTo(new Vector2d(30, -90))
                .strafeTo(new Vector2d(105, -90))
                .strafeTo(new Vector2d(105, -108))
                .strafeTo(new Vector2d(6.3, -100))

                .setTangent(0) //second spec
                .splineToSplineHeading(new Pose2d(61, 0, Math.toRadians(0)), Math.toRadians(0))

                .setTangent(Math.toRadians(180)) //third spec pickup
                .splineToSplineHeading(new Pose2d(6.3, -85, Math.toRadians(180)), Math.toRadians(180))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}