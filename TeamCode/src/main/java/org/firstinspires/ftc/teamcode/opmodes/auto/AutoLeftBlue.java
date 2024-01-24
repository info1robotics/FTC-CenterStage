package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BLUE;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_RED;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_STACK;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.TILE_SIZE;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.AutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class AutoLeftBlue extends AutoBase {
    static Pose2d startPose = new Pose2d(TILE_SIZE * 0.5 + 3.15, TILE_SIZE * 3 - 18, HEADING_TO_RED);

    static TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(3)
    ));


    @Override
    public void onInit() {
        super.onInit();
        claw.close();
        startPos = Pos.BLUE_LEFT;
        drive.setPoseEstimate(startPose);

        TrajectorySequence detectionRight = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(9, 27, HEADING_TO_BACKDROP), HEADING_TO_STACK)
                .relativeTemporalMarker(-0.43, () -> {
                    intake.setPower(1);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(49, 24.0), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(210, 1);
                })
                .build();

        TrajectorySequence detectionMid = drive.trajectorySequenceBuilder(startPose)
                .addSpatialMarker(new Vector2d(27.5, 19.3), () -> {
                    intake.setPower(1);
                })
                .splineTo(new Vector2d(33.5, 18.3), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(48, 33), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(270, 1);
                })
                .build();

        TrajectorySequence detectionLeft = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(slowConstraint)
                .splineToSplineHeading(new Pose2d(28.5, 40.5, HEADING_TO_BLUE), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.37, () -> {
                    intake.setPower(0.8);
                })
                .relativeTemporalMarker(0.3, () -> {
                    intake.setPower(0);
                })
                .resetConstraints()
                .splineToSplineHeading(new Pose2d(52, 35.8, HEADING_TO_BACKDROP), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(270, 1);
                })
                .build();


        Pose2d[] detectionEnds = {
                detectionLeft.end(),
                detectionMid.end(),
                detectionRight.end()
        };

        ArrayList<TrajectorySequence> stackTrajectories = new ArrayList<>();

        for (int i = 0; i < 3; i++) {
            final int finalI = i;
            stackTrajectories.add(drive.trajectorySequenceBuilder(detectionEnds[i])
                    .relativeTemporalMarker(0, () -> {
//                        if (finalI == 0) {
//                            drive.setPoseEstimate(new Pose2d(
//                                    detectionEnds[0].getX(),
//                                    detectionEnds[0].getY(),
//                                    detectionEnds[0].getHeading())
//                            );
//                        }
                        if (finalI == 1) {
                            drive.setPoseEstimate(new Pose2d(
                                    detectionEnds[1].getX() + 2,
                                    detectionEnds[1].getY() - 1,
                                    detectionEnds[1].getHeading())
                            );
                        }
                        else if (finalI == 2) {
                            drive.setPoseEstimate(new Pose2d(
                                    detectionEnds[2].getX(),
                                    detectionEnds[2].getY() - 1.2,
                                    detectionEnds[2].getHeading())
                            );
                        }
                    })
                    .setReversed(true)
                    .splineTo(new Vector2d(25, 8), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(0, 8), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-20, 4.4), Math.toRadians(180))
                    .relativeTemporalMarker(0, () -> {
                        pivotIntake.setPosLeft(0.325);
                    })
                    .setVelConstraint(slowConstraint)
                    .splineToSplineHeading(new Pose2d(-55.1, 3, HEADING_TO_BACKDROP), Math.toRadians(-180))
                    .relativeTemporalMarker(-0.7, () -> {
                        intake.setPower(-1);
                    })
                    .relativeTemporalMarker(0, () -> {
                        pivotIntake.setPosLeft(0.24);
                    })
                    .forward(3)
                    .build());
        }

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(stackTrajectories.get(0).end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(30, 11.5, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.2, () -> {
                    lift.setTargetPosition(500, 1);
                })
                .splineToConstantHeading(new Vector2d(49, 30), Math.toRadians(HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toBackdrop.end())
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(-15, 1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(18, 8), Math.toRadians(180))
                .splineTo(new Vector2d(0, 5), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20, 5), Math.toRadians(180))
                .relativeTemporalMarker(-0.5, () -> {
                    intake.setPower(-1);
                })
                .setVelConstraint(slowConstraint)
                .splineToSplineHeading(new Pose2d(-52, 5, HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.15);
                })
                .lineToLinearHeading(new Pose2d(-52.3, 5, HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence toBackdrop2 = drive.trajectorySequenceBuilder(toStack2.end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                    pivotIntake.setInit();
                })
                .lineToSplineHeading(new Pose2d(20, 5, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0, () -> {
                    lift.setTargetPosition(500, 1);
                })
                .splineToConstantHeading(new Vector2d(49.5, 30), Math.toRadians(HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(toBackdrop2.end())
                .relativeTemporalMarker(0.2, () -> {
                    claw.open();
                    lift.setTargetPosition(590, 1);
                })
                .relativeLineToLinearHeading(new Pose2d(-7, -12, HEADING_TO_RED))
                .build();


        task = serial(
                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.LEFT, trajectorySequence(detectionLeft)),
                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.CENTER, trajectorySequence(detectionMid)),
                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.RIGHT, trajectorySequence(detectionRight)),
                parallel(
                        serial(
                                execute(() -> claw.open()),
                                sleepms(540),
                                execute(() -> lift.setTargetPosition(-15, 1))
                        ),
                        conditional(() -> full, serial(
                                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.LEFT, trajectorySequence(stackTrajectories.get(0))),
                                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.CENTER, trajectorySequence(stackTrajectories.get(1))),
                                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.RIGHT, trajectorySequence(stackTrajectories.get(2))),
                                sleepms(150),
//                                execute(() -> pivotIntake.setPosLeft(0.24)),
//                                sleepms(1000),
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
