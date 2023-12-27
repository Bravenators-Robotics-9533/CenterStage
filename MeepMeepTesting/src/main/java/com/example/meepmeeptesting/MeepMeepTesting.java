package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static final double WIDTH = 16.6;
    private static final double HEIGHT = 17.0;

    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setConstraints(31.449374934056326, 31.449374934056326, 1.677, Math.toRadians(60), 15.0)
                .setDimensions(WIDTH, HEIGHT)
                .followTrajectorySequence(drive ->
                   drive.trajectorySequenceBuilder(new Pose2d(24, -14, Math.toRadians(180)))
                           .splineToConstantHeading(new Vector2d(27, -10), Math.PI * 3 / 2)
                           .lineToLinearHeading(new Pose2d(50, -10, Math.toRadians(180)))
//                           .splineTo(new Vector2d(31.98, -20.86), Math.toRadians(225.00))
//                           .splineToConstantHeading(new Vector2d(27, -10), Math.toRadians(37))

                           .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }

}