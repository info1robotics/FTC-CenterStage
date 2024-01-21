package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.Type.DETECTION;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.Type.FULL;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.conditional;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.trajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.firstinspires.ftc.teamcode.common.AutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoRightBlue extends AutoBase {
    static Pose2d startPose = new Pose2d(-TILE_SIZE * 1.5 - 4.15, TILE_SIZE * 3 - 18, HEADING_TO_RED);
    static TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(3)
    ));

    static TrajectoryVelocityConstraint verySlowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(30),
            new AngularVelocityConstraint(2)
    ));

    AutoConstants.Type type = FULL;


    @Override
    public void onInitTick() {
        super.onInitTick();
        if (gamepad1.a) {
            type = DETECTION;
        } else if (gamepad1.b) {
            type = FULL;
        }

        telemetry.addLine("Right claw closes with yellow pixel.");
        telemetry.addLine("Press A / X for detection only.");
        telemetry.addLine("Press B / O for full auto.");
        if (type == DETECTION) {
            telemetry.addLine("Currently set to run: Detection Only.");
        } else if (type == FULL) {
            telemetry.addLine("Currently set to run: Full Auto.");
        }
    }

    @Override
    public void onInit() {
        super.onInit();

        TrajectorySequence detectionLeft = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-TILE_SIZE - 4, 30, HEADING_TO_STACK), Math.toRadians(HEADING_TO_RED))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(-TILE_SIZE - 12, 8), HEADING_TO_BACKDROP)
                .lineToSplineHeading(new Pose2d(15, 8, HEADING_TO_BACKDROP))
                .splineToConstantHeading(new Vector2d(50, 38), Math.toRadians(HEADING_TO_BACKDROP))
                .resetConstraints()
                .build();

        TrajectorySequence detectionMid = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-18, 8), HEADING_TO_BACKDROP)
                .lineToSplineHeading(new Pose2d(15, 8, HEADING_TO_BACKDROP))
                .splineToConstantHeading(new Vector2d(50, 38), Math.toRadians(HEADING_TO_BACKDROP))
                .resetConstraints()
                .build();

        TrajectorySequence detectionRight = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-16.5 - TILE_SIZE, 10), Math.toRadians(HEADING_TO_RED))
                .splineToConstantHeading(new Vector2d(16, 5), HEADING_TO_BACKDROP)
                .setVelConstraint(verySlowConstraint)
                .splineToConstantHeading(new Vector2d(50, 25), HEADING_TO_BACKDROP)
                .resetConstraints()
                .build();

        Pose2d[] detectionEnds = {
                detectionLeft.end(),
                detectionMid.end(),
                detectionRight.end()
        };

        ArrayList<TrajectorySequence> stackTrajectories = new ArrayList<>();

        for (int i = 0; i < 3; i++) {
            stackTrajectories.add(drive.trajectorySequenceBuilder(detectionEnds[0])
                    .setReversed(true)
                    .splineTo(new Vector2d(13, 2), Math.toRadians(180))
                    .splineTo(new Vector2d(0, 2), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-20, 5), Math.toRadians(180))
                    .relativeTemporalMarker(0, () -> {
                        pivotIntake.setPosLeft(0.091);
                    })
                    .setVelConstraint(slowConstraint)
                    .splineToSplineHeading(new Pose2d(-56, 4.8, Math.toRadians(-25) + HEADING_TO_BACKDROP), Math.toRadians(-180))
                    .relativeTemporalMarker(0, () -> {
                        new Thread(() -> {
                            try {
                                Thread.sleep(1400);
                                intake.setPower(-1);
                            } catch (Exception ignored) {

                            }
                        }).start();
                    })
                    .lineToSplineHeading(new Pose2d(-57, 4, Math.toRadians(0) + HEADING_TO_BACKDROP))
                    .back(2)
                    .build());
        }

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(stackTrajectories.get(0).end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(10, 9, HEADING_TO_BACKDROP))
                .setVelConstraint(slowConstraint)
                .relativeTemporalMarker(-0.5, () -> {
                    lift.setTargetPosition(500, 1);
                })
                .splineToConstantHeading(new Vector2d(49.2, 30), Math.toRadians(HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toBackdrop.end())
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(-15, 1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(13, 3), Math.toRadians(180))
                .splineTo(new Vector2d(0, 4), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20, 0), Math.toRadians(180))
                .setVelConstraint(slowConstraint)
                .splineToSplineHeading(new Pose2d(-55, 2.5, HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(-0.4, () -> {
                    pivotIntake.setPosLeft(0.04);
                    intake.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-54.7, 0.9, Math.toRadians(19) + HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence toBackdrop2 = drive.trajectorySequenceBuilder(toStack2.end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(10, 2, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.3, () -> {
                    lift.setTargetPosition(500, 1);
                })
                .splineToConstantHeading(new Vector2d(50, 29.75), Math.toRadians(HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(toBackdrop2.end())
                .relativeTemporalMarker(0.2, () -> {
                    claw.open();
                    lift.setTargetPosition(590, 1);
                })
                .relativeLineToLinearHeading(new Pose2d(-5, -20, HEADING_TO_RED))
                .build();

        task = serial(
                conditional(() -> detectedZone == AutoConstants.TSEPosition.LEFT, trajectorySequence(detectionLeft)),
                conditional(() -> detectedZone == AutoConstants.TSEPosition.CENTER, trajectorySequence(detectionMid)),
                conditional(() -> detectedZone == AutoConstants.TSEPosition.RIGHT, trajectorySequence(detectionRight)),
                parallel(
                        serial(
                                execute(() -> claw.open()),
                                sleepms(540),
                                execute(() -> lift.setTargetPosition(-15, 1))
                        ),
                        conditional(() -> type == FULL, serial(
                                conditional(() -> detectedZone == AutoConstants.TSEPosition.LEFT, trajectorySequence(stackTrajectories.get(0))),
                                conditional(() -> detectedZone == AutoConstants.TSEPosition.CENTER, trajectorySequence(stackTrajectories.get(1))),
                                conditional(() -> detectedZone == AutoConstants.TSEPosition.RIGHT, trajectorySequence(stackTrajectories.get(2))),
                                sleepms(150),
                                execute(() -> pivotIntake.setPosLeft(0.07)),
                                sleepms(1000),
                                execute(() -> {
                                    intake.setPower(0);
                                    claw.close();
                                }),
                                trajectorySequence(toBackdrop),
                                sleepms(200),
                                execute(() -> {
                                    claw.open();
                                    lift.setTargetPosition(550, 1);
                                }),
                                trajectorySequence(toStack2),
                                sleepms(1000),
                                execute(() -> {
                                    intake.setPower(0);
                                    claw.close();
                                }),
                                trajectorySequence(toBackdrop2)
                        ))
                ),
                trajectorySequence(park),
                execute(() -> {
                    lift.setTargetPosition(-20, 1);
                    claw.open();
                }),
                sleepms(200)
        );
    }
}
