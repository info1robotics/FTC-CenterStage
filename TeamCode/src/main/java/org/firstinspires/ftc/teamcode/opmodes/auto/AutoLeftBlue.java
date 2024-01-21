package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.HEADING_TO_RED;
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

import java.util.Arrays;

@Autonomous
public class AutoLeftBlue extends AutoBase {
    static Pose2d startPose = new Pose2d(TILE_SIZE * 0.5 + 4.15, TILE_SIZE * 3 - 18, HEADING_TO_RED);

    static TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(3)
    ));

    static TrajectoryVelocityConstraint verySlowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(30),
            new AngularVelocityConstraint(2)
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
        telemetry.addData("Detection Case", detectedZone);
    }

    @Override
    public void onInit() {
        startPos = Pos.BLUE_LEFT;
        super.onInit();
        claw.close();
        pivot.setCollect();
        drive.setPoseEstimate(startPose);

        TrajectorySequence detectRight = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(16.5, 22), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.43, () -> {
                    intake.setPower(0.8);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(49.2, 24), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-2, () -> {
                    lift.setTargetPosition(300, 1);
                })
                .relativeTemporalMarker(-1, () -> {
                    pivot.setDrop();
                })
                .build();

        TrajectorySequence toStackRight = drive.trajectorySequenceBuilder(detectRight.end())
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
                .build();

        TrajectorySequence detectMid = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(23.5, 18.5), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-0.36, () -> {
                    intake.setPower(0.5);
                })
                .relativeTemporalMarker(0.6, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(48.2, 31.5), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(-2, () -> {
                    lift.setTargetPosition(300, 1);
                })
                .relativeTemporalMarker(-1, () -> {
                    pivot.setDrop();
                })
                .build();

        TrajectorySequence toStackMid = drive.trajectorySequenceBuilder(detectMid.end())
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
                .build();

        TrajectorySequence detectLeft = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(slowConstraint)
                .splineTo(new Vector2d(22.5, 26), Math.toRadians(HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0.1, () -> {
                    intake.setPower(0.8);
                })
                .relativeTemporalMarker(0.6, () -> {
                    intake.setPower(0);
                })
                .setVelConstraint(verySlowConstraint)
                .splineToConstantHeading(new Vector2d(50, 39.0), Math.toRadians(70))
                .resetConstraints()
                .relativeTemporalMarker(-2, () -> {
                    lift.setTargetPosition(300, 1);
                })
                .relativeTemporalMarker(-1, () -> {
                    pivot.setDrop();
                })
                .build();

        TrajectorySequence toStackLeft = drive.trajectorySequenceBuilder(detectLeft.end())
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
                .build();

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(toStackMid.end())
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(1);
                })
                .addTemporalMarker(2, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(10, 9, HEADING_TO_BACKDROP))
                .setVelConstraint(slowConstraint)
                .relativeTemporalMarker(-0.5, () -> {
                    lift.setTargetPosition(300, 1);
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
                    lift.setTargetPosition(300, 1);
                })
                .splineToConstantHeading(new Vector2d(50, 29.75), Math.toRadians(HEADING_TO_BACKDROP))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(toBackdrop2.end())
                .relativeTemporalMarker(0.2, () -> {
                    claw.open();
                    lift.setTargetPosition(590, 1);
                })
                .relativeLineToLinearHeading(new Pose2d(-5, 20, HEADING_TO_RED))
                .build();

        task = serial(
                execute(() -> {
                    if (detectedZone == AutoConstants.TSEPosition.LEFT) {
                        drive.followTrajectorySequence(detectLeft);
                    } else if (detectedZone == AutoConstants.TSEPosition.RIGHT) {
                        drive.followTrajectorySequence(detectRight);
                    } else {
                        drive.followTrajectorySequence(detectMid);
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
