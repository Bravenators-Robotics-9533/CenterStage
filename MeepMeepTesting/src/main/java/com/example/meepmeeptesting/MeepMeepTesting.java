package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static final Pose2d START_POSITION = new Pose2d(12, 65, Math.toRadians(0));


    private static final double WIDTH = 16.6;
    private static final double HEIGHT = 17.0;

    private static final Pose2d OFF_BACKDROP_POS = new Pose2d(35, -26, Math.toRadians(180));


    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setConstraints(31.449374934056326, 31.449374934056326, 1.677, Math.toRadians(60), 15.0)
                .setDimensions(WIDTH, HEIGHT)
                .followTrajectorySequence(drive ->
                   drive.trajectorySequenceBuilder(new Pose2d(27, -10, Math.toRadians(0)))
                           .lineTo(new Vector2d(40, -10))
                           .lineToLinearHeading(new Pose2d(40, -60, Math.toRadians(180)))
                           .lineToConstantHeading(new Vector2d(66, -60))

                           .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }

}