package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SmallAutoBlue {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, 18, 0))
                .turn(Math.toRadians(180))
                .splineTo(new Vector2d(35, 40), Math.PI / 2)
                .lineToY(50)
                .splineTo(new Vector2d(60,16), Math.PI/2)
                .turn(Math.toRadians(120))
                .waitSeconds(3)
                .splineTo(new Vector2d(11,40), Math.PI/2)
                .lineToY(50)
                .splineTo(new Vector2d(60,16), Math.PI/2)
                .turn(Math.toRadians(120))
                .waitSeconds(3)
                .splineTo(new Vector2d(57,35), Math.PI/2)
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}