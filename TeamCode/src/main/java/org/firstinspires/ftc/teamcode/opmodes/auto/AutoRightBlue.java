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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.AutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Autonomous
public class AutoRightBlue extends AutoBase {

    static Pose2d startPose = new Pose2d(-TILE_SIZE * 1.5 - 4.15, TILE_SIZE * 3 - 18, HEADING_TO_RED);

    @Override
    public void onInit() {
        super.onInit();
        claw.open(Claw.Type.LEFT);
        claw.close(Claw.Type.RIGHT);
        startPos = Pos.BLUE_RIGHT;
        drive.setPoseEstimate(startPose);

        TrajectorySequence detectionRight = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 - 10, TILE_SIZE * 1.5 + 3.5, HEADING_TO_BLUE)) // to line
                .relativeTemporalMarker(0, () -> {
                    intake.setPower(0.9);
                })
                .relativeTemporalMarker(0.2, () -> {
                    intake.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(-TILE_SIZE * 2 - 5.5, TILE_SIZE * 1 + 21.5, HEADING_TO_BACKDROP))
                .relativeTemporalMarker(0, () -> {
                    intake.setPower(-1);
                })
                .relativeTemporalMarker(0.2, () -> {
                    pivotIntake.setPosLeft(0.34);
                })
                .splineToLinearHeading(new Pose2d(-TILE_SIZE * 2 - 9.5, TILE_SIZE * 1 + 11.0, HEADING_TO_BACKDROP + Math.toRadians(32)), HEADING_TO_BLUE)
                .waitSeconds(.7)
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.28);
                })
                .relativeTemporalMarker(0.4, () -> {
                    claw.close();
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(1);
                })
                .relativeTemporalMarker(1.3, () -> {
                    intake.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 - 5.15, TILE_SIZE * 2 + 7, HEADING_TO_BACKDROP))
                .lineToConstantHeading(new Vector2d(17, TILE_SIZE * 2 + 7))
                .splineToConstantHeading(new Vector2d(48, TILE_SIZE * 1 + 4), HEADING_TO_BACKDROP)
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(400, 1);
                })
                .relativeTemporalMarker(0.2, () -> {
                    claw.open();
                    lift.setTargetPosition(600, 1);
                })
                .waitSeconds(1)
                .relativeLineToLinearHeading(new Pose2d(-7, TILE_SIZE - 4 + 2, HEADING_TO_RED))
                .build();

        TrajectorySequence detectionMid = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-TILE_SIZE * 1.5 - 3.15, TILE_SIZE * 1.5, HEADING_TO_BLUE), HEADING_TO_RED)
                .relativeTemporalMarker(0, () -> {
                    intake.setPower(0.9);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .splineToConstantHeading(new Vector2d(-TILE_SIZE * 1.5 - 4.15, TILE_SIZE * 1 + 6), HEADING_TO_RED)
                .splineToSplineHeading(new Pose2d(-TILE_SIZE * 1.5 - 4.15, TILE_SIZE * 1 + 12, HEADING_TO_BACKDROP), HEADING_TO_RED)
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.34);
                    intake.setPower(-1);
                })
                .lineToConstantHeading(new Vector2d(-TILE_SIZE * 2 - 8.5, TILE_SIZE * 1 + 3.5)) // to stack
                .waitSeconds(.7)
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.28);
                })
                .relativeTemporalMarker(0.4, () -> {
                    claw.close();
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(1);
                })
                .relativeTemporalMarker(1.3, () -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(-TILE_SIZE * 1.5 - 4.15, TILE_SIZE * 2 + 6.5))
                .lineToConstantHeading(new Vector2d(17, TILE_SIZE * 2 + 6.5))
                .splineToConstantHeading(new Vector2d(48, TILE_SIZE * 1 + 8.5), HEADING_TO_BACKDROP)
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(400, 1);
                })
                .relativeTemporalMarker(0.2, () -> {
                    claw.open();
                    lift.setTargetPosition(600, 1);
                })
                .waitSeconds(1)
                .relativeLineToLinearHeading(new Pose2d(-7, TILE_SIZE - 6, HEADING_TO_RED))
                .build();

        TrajectorySequence detectionLeft = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 - 1.5, TILE_SIZE * 1.5 - 11, HEADING_TO_STACK))
                .relativeTemporalMarker(0.2, () -> {
                    intake.setPower(1);
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(0);
                })
                .relativeTemporalMarker(0.7, () -> {
                    pivotIntake.setPosLeft(0.34);
                    intake.setPower(-1);
                })
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 2 - 8.5, TILE_SIZE * 1 + 7, HEADING_TO_BACKDROP)) // to stack
                .waitSeconds(.7)
                .relativeTemporalMarker(0, () -> {
                    pivotIntake.setPosLeft(0.28);
                })
                .relativeTemporalMarker(0.4, () -> {
                    claw.close();
                })
                .relativeTemporalMarker(0.5, () -> {
                    intake.setPower(1);
                })
                .relativeTemporalMarker(1.3, () -> {
                    intake.setPower(0);
                })
                .lineToConstantHeading(new Vector2d(-TILE_SIZE * 1.5 - 4.15, TILE_SIZE * 2 + 6))
                .lineToConstantHeading(new Vector2d(17, TILE_SIZE * 2 + 6))
                .splineToConstantHeading(new Vector2d(48, TILE_SIZE * 1 + 14), HEADING_TO_BACKDROP)
                .relativeTemporalMarker(-1.2, () -> {
                    lift.setTargetPosition(400, 1);
                })
                .relativeTemporalMarker(0.2, () -> {
                    claw.open();
                    lift.setTargetPosition(600, 1);
                })
                .waitSeconds(1)
                .relativeLineToLinearHeading(new Pose2d(-7, TILE_SIZE - 13, HEADING_TO_RED))
                .build();


        task = serial(
                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.LEFT, trajectorySequence(detectionLeft)),
                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.CENTER, trajectorySequence(detectionMid)),
                conditional(() -> getDetectedZone() == AutoConstants.TSEPosition.RIGHT, trajectorySequence(detectionRight)),
                sleepms(200),
                execute(() -> {
                    claw.open();
                    lift.setTargetPosition(-20, 1);
                })
        );
    }
}
