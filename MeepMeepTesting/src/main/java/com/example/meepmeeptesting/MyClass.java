package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static double TILE_SIZE = 24.0; // inch
    public static double ROBOT_HEIGHT_HALF = 9.84; // inch TODO

    public enum TSEPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public static double HEADING_TO_BLUE = Math.toRadians(90);
    public static double HEADING_TO_RED = Math.toRadians(-90);
    public static double HEADING_TO_BACKDROP = Math.toRadians(0);
    public static double HEADING_TO_STACK = Math.toRadians(180);
    static Pose2d startPose = new Pose2d(TILE_SIZE * 0.5 + 3.15, -(-TILE_SIZE * 3 + 18), HEADING_TO_RED);
    public static void main(String[] args) {


        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
//                                .forward(10)
//                                .splineTo(new Vector2d(16.5, 30), Math.toRadians(HEADING_TO_BACKDROP))
//                                .splineToConstantHeading(new Vector2d(49.5,  21.0), Math.toRadians(HEADING_TO_BACKDROP))
                                .setReversed(true)
                                .splineTo(new Vector2d(13, 7), Math.toRadians(180))
                                .splineTo(new Vector2d(0, 8), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-20, 4.8), Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
//                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .start();
    }
}