package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.74, 66.05, Math.toRadians(270.00)))
                                .splineTo(new Vector2d(-36.06, 50.94), Math.toRadians(-87.40))
                                .splineTo(new Vector2d(-35.83, 34.45), Math.toRadians(6.01))
                                .splineTo(new Vector2d(-28.27, 32.85), Math.toRadians(-11.98))
                                .splineTo(new Vector2d(-37.20, 32.62), Math.toRadians(-80.84))
                                .splineTo(new Vector2d(-34.68, 17.51), Math.toRadians(-80.54))
                                .splineTo(new Vector2d(-35.37, 8.36), Math.toRadians(265.71))
                                .splineTo(new Vector2d(-23.24, 10.19), Math.toRadians(8.58))
                                .splineTo(new Vector2d(-5.61, 10.19), Math.toRadians(0.00))
                                .splineTo(new Vector2d(12.02, 10.19), Math.toRadians(0.00))
                                .splineTo(new Vector2d(48.65, 11.33), Math.toRadians(1.79))
                                .splineTo(new Vector2d(42.70, 36.74), Math.toRadians(95.34))
                                .splineTo(new Vector2d(49.11, 36.06), Math.toRadians(3.81))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}