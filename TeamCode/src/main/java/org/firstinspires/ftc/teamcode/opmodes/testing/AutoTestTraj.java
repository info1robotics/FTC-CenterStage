package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous
public class AutoTestTraj extends AutoBase {
    TrajectorySequence toStack;
    static Pose2d startPose = new Pose2d(TILE_SIZE * 0.5 + 3.15, -TILE_SIZE * 3 + 18, Math.toRadians(90));

    static TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(3)
    ));

    @Override
    public void onInit() {
        super.onInit();
        claw.close();
        drive.setPoseEstimate(startPose);
        toStack = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(23.5,-20), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.43, () -> {
                    intake.setPower(0.5);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(51,-32.4), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(230, 1);
                })
                .relativeTemporalMarker(0.14, () -> {
                    claw.open();
                })
                .relativeTemporalMarker(1.14, () -> {
                    lift.setTargetPosition(-5, 1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(13, -7), Math.toRadians(180))
                .splineTo(new Vector2d(0, -8), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20, -4.8), Math.toRadians(180))
//                .lineTo(new Vector2d(-25, -8.9))
                .setVelConstraint(slowConstraint)
//                .lineTo(new Vector2d(-53, -8.9)) // 0.13 pivot intake
                .splineToSplineHeading(new Pose2d(-54.1, -4.8, Math.toRadians(32) + HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.120);
                })
                .turn(Math.toRadians(-39.5), 3, 3)
                .relativeTemporalMarker(0, () -> {
                    intake.setPower(-1);
                })
                .back(3.7)
                .build();

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(toStack.end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(1, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(20, -9, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0, () -> {
                    lift.setTargetPosition(500, 1);
                })
                .splineToConstantHeading(new Vector2d(51.5,-35), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0, () -> {
                    claw.open();
                    lift.setTargetPosition(550, 1);
                })
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toBackdrop.end())
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(-10, 1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(13, -7), Math.toRadians(180))
                .splineTo(new Vector2d(0, -8), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20, 0), Math.toRadians(180))
//                .lineTo(new Vector2d(-25, -8.9))
                .setVelConstraint(slowConstraint)
//                .lineTo(new Vector2d(-53, -8.9)) // 0.13 pivot intake
                .splineToSplineHeading(new Pose2d(-52.2, 0, HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.09);
                    intake.setPower(-1);
                })
                .back(4.7)
                .build();

        TrajectorySequence toBackdrop2 = drive.trajectorySequenceBuilder(toStack2.end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(1, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(20, -2, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0, () -> {
                    lift.setTargetPosition(500, 1);
                })
                .splineToConstantHeading(new Vector2d(51.5,-35), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0, () -> {
                    claw.open();
                    lift.setTargetPosition(550, 1);
                })
                .build();

        task = serial(
                trajectorySequence(toStack),
                sleepms(350),
                execute(() -> pivotIntake.setPosLeft(0.090)),
                sleepms(1000),
                execute(() -> {
                    intake.setPower(0);
                    claw.close();
                }),
                trajectorySequence(toBackdrop),
                trajectorySequence(toStack2),
                sleepms(1000),
                execute(() -> {
                    intake.setPower(0);
                    claw.close();
                }),
                trajectorySequence(toBackdrop2)
        );
    }

}
