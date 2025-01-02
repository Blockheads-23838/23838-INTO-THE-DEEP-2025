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
                .setConstraints(160, 150, Math.toRadians(180) * 1.5, Math.toRadians(180) * 1.5, 18)
                .build();

        //X-OFFSET -> -62; Y-OFFSET -> +50
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0 - 62, 0 + 50, Math.toRadians(0)))
                //.waitSeconds(3)

                .strafeTo(new Vector2d(61 - 62, 0 + 50)) //preload

                .waitSeconds(0.7)

                .setTangent(Math.toRadians(180)) //pushticks
                .splineToSplineHeading(new Pose2d(46 - 62, -30 + 50, Math.toRadians(90)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(102 - 62, -57 + 50, Math.toRadians(180)), Math.toRadians(0))
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(107 - 62, -64 + 50), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(105 - 62, -71 + 50), Math.toRadians(180))
                .strafeTo(new Vector2d(30 - 62, -71 + 50))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(105 - 62, -71 + 50), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(108 - 62, -80 + 50), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(105 - 62, -90 + 50), Math.toRadians(180))
                .strafeTo(new Vector2d(30 - 62, -90 + 50))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(105 - 62, -90 + 50), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(108 - 62, -99 + 50), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(105 - 62, -108 + 50), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30 - 62, -103 + 50), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6.3 - 62, -85 + 50), Math.toRadians(180))

                .waitSeconds(0.3)

                .setReversed(true) //second spec
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(61 - 62, 0 + 50, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.7)

                .setReversed(false) //third spec pickup
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(6.3 - 62, -85 + 50, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(0.3)

                .setReversed(true) //third spec
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(61 - 62, 3 + 50, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.7)

                .setReversed(false) //fourth spec pickup
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(6.3 - 62, -85 + 50, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(0.3)

                .setReversed(true) //fourth spec
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(61 - 62, 6 + 50, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.7)

                .setReversed(false) //fifth spec pickup
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(6.3 - 62, -85 + 50, Math.toRadians(180)), Math.toRadians(180))

                .waitSeconds(0.3)

                .setReversed(true) //fifth spec
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(61 - 62, 9 + 50, Math.toRadians(0)), Math.toRadians(0))

                .waitSeconds(0.7)

                .build());

        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}