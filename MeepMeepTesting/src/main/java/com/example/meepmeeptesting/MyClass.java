package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MyClass {
    public static double TILE_SIZE = 24.0; // inch
    public static double ROBOT_HEIGHT_HALF = 9.84; // inch TODO

    public enum TSEPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public static Vector2d v(double x, double y) {
        return new Vector2d(x, y);
    }

    public static Pose2d p(double x, double y, double theta) {
        return new Pose2d(x, y, theta);
    }

    public static double rad(double angdeg) {
        return Math.toRadians(angdeg);
    }

    public static double HEADING_TO_BLUE = Math.toRadians(90);
    public static double HEADING_TO_RED = Math.toRadians(-90);
    public static double HEADING_TO_BACKDROP = Math.toRadians(0);
    public static double HEADING_TO_STACK = Math.toRadians(180);

    static TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(3)
    ));

    static TrajectoryVelocityConstraint verySlowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(30),
            new AngularVelocityConstraint(2)
    ));

    static Pose2d startPose = p(49, 31.6, HEADING_TO_BACKDROP);
    public static void main(String[] args) {




        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .setReversed(true)
                                .splineToSplineHeading(p(17, 53, HEADING_TO_BACKDROP), rad(180))
                                .splineToSplineHeading(p(-17.85, 55, HEADING_TO_BACKDROP + rad(30)), rad(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myFirstBot)
                .start();
    }
}