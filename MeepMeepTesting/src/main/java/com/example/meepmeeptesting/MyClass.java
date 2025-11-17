package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-54, -46, 0))
                .turn(Math.toRadians(55))
                .lineToX(-40)
//                .splineTo(new Vector2d(-33, -33), Math.PI / 2)
                .splineTo(new Vector2d(-12, -25), Math.PI / 2)
                .turn(Math.toRadians(180))
                .lineToY(-50)
                .splineTo(new Vector2d(-25,-28), Math.PI/2)
                .turn(Math.toRadians(30))
                .waitSeconds(3)
                .splineTo(new Vector2d(10,-27), Math.PI/2)
                .turn(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-25, 28, 250), Math.PI / 2)
                .lineToY(-50)
                .splineTo(new Vector2d(-25,-28), Math.PI/2)
                .turn(Math.toRadians(30))
                .waitSeconds(3)
                .splineTo(new Vector2d(-12,-35), Math.PI/2)

                .waitSeconds(3)


                //24 seconds
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}