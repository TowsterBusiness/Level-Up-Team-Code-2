package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
//-35.5, 59
        //-45.5, 35
        //≈
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(180)))
                                        .splineToConstantHeading(new Vector2d(-10, -24), Math.toRadians(270))
                                        .lineTo(new Vector2d(-10, -30))
                                        .setTangent(Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(270))
                                        .lineTo(new Vector2d(0, -36))
                                        .lineToLinearHeading(new Pose2d(0, -27, Math.toRadians(270)))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}