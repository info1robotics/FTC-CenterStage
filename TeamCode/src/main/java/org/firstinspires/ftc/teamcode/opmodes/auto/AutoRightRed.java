package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BLUE;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.Type.DETECTION;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.Type.FULL;
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

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class AutoRightRed extends AutoBase {
    static Pose2d startPose = new Pose2d(TILE_SIZE * 0.5 + 3.15, -TILE_SIZE * 3 + 18, Math.toRadians(90));

    static TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(3)
    ));

    AutoConstants.Type type = DETECTION;


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
        startPos = Pos.RED_RIGHT;
        super.onInit();
        claw.close();
        pivot.setCollect();
        drive.setPoseEstimate(startPose);

        TrajectorySequence detectionLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(15.5, -25), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.33, () -> {
                    intake.setPower(0.8);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50.8, -21.0), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(300, 1);
                })
                .relativeTemporalMarker(-0.8, () -> {
                    pivot.setDrop();
                })
                .build();

        TrajectorySequence detectionMid = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(23.5, -19.3), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.43, () -> {
                    intake.setPower(0.5);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(50.8, -28.5), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(300, 1);
                })
                .relativeTemporalMarker(-0.8, () -> {
                    pivot.setDrop();
                })
                .build();

        TrajectorySequence detectionRight = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(slowConstraint)
                .splineTo(new Vector2d(37.5, -30), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.52, () -> {
                    intake.setPower(0.8);
                })
                .relativeTemporalMarker(0.3, () -> {
                    intake.setPower(0);
                })
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(50, -35.8), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(300, 1);
                })
                .relativeTemporalMarker(-0.8, () -> {
                    pivot.setDrop();
                })
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
                    .splineTo(new Vector2d(13, -7), Math.toRadians(180))
                    .splineTo(new Vector2d(0, -8), Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-20, -4.8), Math.toRadians(180))
                    .relativeTemporalMarker(0, () -> {
                        pivotIntake.setPosLeft(0.1);
                    })
                    .setVelConstraint(slowConstraint)
                    .splineToSplineHeading(new Pose2d(-56, -4.8, Math.toRadians(25) + HEADING_TO_BACKDROP), Math.toRadians(-180))
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
                    .lineToSplineHeading(new Pose2d(-57.2, 0, Math.toRadians(0) + HEADING_TO_BACKDROP))
//                    .back(1.2)
                    .build());
        }

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(stackTrajectories.get(0).end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                    pivotIntake.setInit();
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
                .setVelConstraint(slowConstraint)
                .splineToSplineHeading(new Pose2d(-55, -4, HEADING_TO_BACKDROP), Math.toRadians(-180))
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.04);
                    intake.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-56.2, -0, Math.toRadians(-13) + HEADING_TO_BACKDROP))
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
                .splineToConstantHeading(new Vector2d(51.6, -30), Math.toRadians(HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(toBackdrop2.end())
                .relativeTemporalMarker(0.2, () -> {
                    claw.open();
                    lift.setTargetPosition(300, 1);
                })
                .relativeLineToLinearHeading(new Pose2d(-7, -20, HEADING_TO_BLUE))
                .build();

        task = serial(
                execute(() -> {
                    if (detectedZone == AutoConstants.TSEPosition.LEFT) {
                        drive.followTrajectorySequence(detectionLeft);
                    } else if (detectedZone == AutoConstants.TSEPosition.RIGHT) {
                        drive.followTrajectorySequence(detectionRight);
                    } else {
                        drive.followTrajectorySequence(detectionMid);
                    }
                }),
                execute(() -> claw.open()),
                sleepms(540),
                execute(() -> lift.setTargetPosition(-15, 1)),
                trajectorySequence(park),
                execute(() -> {
                    lift.setTargetPosition(-20, 1);
                    claw.open();
                }),
                sleepms(200)
        );
    }

}
