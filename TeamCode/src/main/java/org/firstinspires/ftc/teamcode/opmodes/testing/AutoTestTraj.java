package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BLUE;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.conditional;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.execute;
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
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightRed;

import java.util.Arrays;

@Autonomous
public class AutoTestTraj extends AutoBase {
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

        TrajectorySequence toStackLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(16.5, -30), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.43, () -> {
                    intake.setPower(0.8);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(49.5, -21.0), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(210, 1);
                })
                .relativeTemporalMarker(0.0, () -> {
                    claw.open();
                })
                .relativeTemporalMarker(0.54, () -> {
                    lift.setTargetPosition(-15, 1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(13, -7), Math.toRadians(180))
                .splineTo(new Vector2d(0, -8), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20, -4.8), Math.toRadians(180))
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.100);
                })
                .setVelConstraint(slowConstraint)
                .splineToSplineHeading(new Pose2d(-54.3, -4.8, Math.toRadians(35) + HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(0, () -> {
                    new Thread(() -> {
                        try {
                            Thread.sleep(1400);
                            intake.setPower(-1);
                        } catch (Exception ignored) {

                        }
                    }).start();
                })
                .lineToSplineHeading(new Pose2d(-56.7, -4, Math.toRadians(0) + HEADING_TO_BACKDROP))
                .back(1.2)
                .build();

        TrajectorySequence toStackMid = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(23.5, -20), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.43, () -> {
                    intake.setPower(0.5);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(49.5, -28.5), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(210, 1);
                })
                .relativeTemporalMarker(0.0, () -> {
                    claw.open();
                })
                .relativeTemporalMarker(1.14, () -> {
                    lift.setTargetPosition(-15, 1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(13, -7), Math.toRadians(180))
                .splineTo(new Vector2d(0, -8), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20, -4.8), Math.toRadians(180))
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.100);
                })
                .setVelConstraint(slowConstraint)
                .splineToSplineHeading(new Pose2d(-54.3, -4.8, Math.toRadians(35) + HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(0, () -> {
                    new Thread(() -> {
                        try {
                            Thread.sleep(1400);
                            intake.setPower(-1);
                        } catch (Exception ignored) {

                        }
                    }).start();
                })
                .lineToSplineHeading(new Pose2d(-56.7, -4, Math.toRadians(0) + HEADING_TO_BACKDROP))
                .back(1.2)
                .build();

        TrajectorySequence toStackRight = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(slowConstraint)
                .splineTo(new Vector2d(37.5, -30), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.37, () -> {
                    intake.setPower(0.8);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(49.5, -36.0), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(210, 1);
                })
                .relativeTemporalMarker(0.0, () -> {
                    claw.open();
                })
                .relativeTemporalMarker(0.54, () -> {
                    lift.setTargetPosition(-15, 1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(13, -7), Math.toRadians(180))
                .splineTo(new Vector2d(0, -8), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20, -4.8), Math.toRadians(180))
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.100);
                })
                .setVelConstraint(slowConstraint)
                .splineToSplineHeading(new Pose2d(-54.3, -4.8, Math.toRadians(35) + HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(0, () -> {
                    new Thread(() -> {
                        try {
                            Thread.sleep(1400);
                            intake.setPower(-1);
                        } catch (Exception ignored) {

                        }
                    }).start();
                })
                .lineToSplineHeading(new Pose2d(-56.7, -4, Math.toRadians(0) + HEADING_TO_BACKDROP))
                .back(1.2)
                .build();

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(toStackMid.end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(30, -9, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.5, () -> {
                    lift.setTargetPosition(500, 1);
                })
                .splineToConstantHeading(new Vector2d(52, -30), Math.toRadians(HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toBackdrop.end())
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(-15, 1);
                })
                .setReversed(true)
                .splineTo(new Vector2d(13, -7), Math.toRadians(180))
                .splineTo(new Vector2d(0, -8), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-20, 0), Math.toRadians(180))
//                .lineTo(new Vector2d(-25, -8.9))
                .setVelConstraint(slowConstraint)
//                .lineTo(new Vector2d(-53, -8.9)) // 0.13 pivot intake
                .splineToSplineHeading(new Pose2d(-54.2, -4, HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.04);
                    intake.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-55, -0, Math.toRadians(-13) + HEADING_TO_BACKDROP))
//                .back(2.7)
                .build();

        TrajectorySequence toBackdrop2 = drive.trajectorySequenceBuilder(toStack2.end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(20, -2, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0, () -> {
                    lift.setTargetPosition(500, 1);
                })
                .splineToConstantHeading(new Vector2d(51.7, -30), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.2, () -> {
                    claw.open();
                    lift.setTargetPosition(550, 1);
                })
                .relativeTemporalMarker(-1, () -> {
                    claw.open();
                    lift.setTargetPosition(-20, 1);
                })
                .relativeLineToLinearHeading(new Pose2d(-7, 12, HEADING_TO_BLUE))
                .build();

        task = serial(
                conditional(() -> detectedZone == AutoConstants.TSEPosition.LEFT, trajectorySequence(toStackLeft)),
                conditional(() -> detectedZone == AutoConstants.TSEPosition.CENTER, trajectorySequence(toStackMid)),
                conditional(() -> detectedZone == AutoConstants.TSEPosition.RIGHT, trajectorySequence(toStackRight)),
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
                trajectorySequence(toBackdrop2),
                execute(() -> {
                    lift.setTargetPosition(-20, 1);
                    claw.open();
                }),
                sleepms(200)
        );
    }

}
