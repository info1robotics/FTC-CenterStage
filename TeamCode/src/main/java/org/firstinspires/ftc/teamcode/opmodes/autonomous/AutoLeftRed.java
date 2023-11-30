package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.ROBOT_HEIGHT_HALF;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.conditional;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.trajectory;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.trajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.AutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineLeftRed;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightBlue;

@Autonomous
public class AutoLeftRed extends AutoBase {
    Pose2d startPose = new Pose2d(-TILE_SIZE * 1.5, -TILE_SIZE * 3 + ROBOT_HEIGHT_HALF, Math.toRadians(90));
    Pose2d alignStack = new Pose2d(-TILE_SIZE * 1.5 - 3, -12, AutoConstants.HEADING_TO_BACKDROP);

    AutoConstants.TSEPosition pos = AutoConstants.TSEPosition.RIGHT;

    @Override
    public void onInitTick() {
        pos = ((TSEDetectionPipelineLeftRed) pipeline).getAnalysis();
        claw.close(Claw.Type.LEFT);
        claw.open(Claw.Type.RIGHT);
        telemetry.addData("pos", pos.toString());

    }

    @Override
    public void onInit() {
        startPos = Pos.RED_LEFT;
        pivotIntake.setInit();
        drive.setPoseEstimate(startPose);

        Trajectory rightToLine = drive.trajectoryBuilder(startPose) //
                .splineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 + 7.3, -TILE_SIZE * 1.5 + 1.1, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence rightAlignStack = drive.trajectorySequenceBuilder(rightToLine.end()) //
                .lineToConstantHeading(new Vector2d(rightToLine.end().getX() - 10, rightToLine.end().getY()))
                .lineToLinearHeading(alignStack)
                .build();

        Trajectory middleToLine = drive.trajectoryBuilder(startPose) // ?
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 + 1, -TILE_SIZE + 9.6, AutoConstants.HEADING_TO_BLUE))
                .build();

        TrajectorySequence middleAlignStack = drive.trajectorySequenceBuilder(middleToLine.end())
                .lineToConstantHeading(new Vector2d(- TILE_SIZE * 1.5 - 9, 0))
                .lineToLinearHeading(alignStack)
                .build();

        Trajectory leftToLine = drive.trajectoryBuilder(startPose) // /
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 2 + 5.3, -TILE_SIZE * 1, Math.toRadians(90)))
                .build();

        Trajectory leftAlignStack = drive.trajectoryBuilder(leftToLine.end()) //
                .lineToLinearHeading(alignStack)
                .build();

        Trajectory toStack = drive.trajectoryBuilder(leftAlignStack.end())
                .lineToConstantHeading(new Vector2d(-TILE_SIZE * 2 - 8, -TILE_SIZE / 2))
                .build();

        Trajectory toAlign = drive.trajectoryBuilder(leftAlignStack.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5, -TILE_SIZE / 2 + 3.5))
                .build();

        Trajectory leftToBackdrop = drive.trajectoryBuilder(toAlign.end()) //
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 17.4, -TILE_SIZE - 15.2))
                .build();

        Trajectory middleToBackdrop = drive.trajectoryBuilder(toAlign.end()) //
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 18, -TILE_SIZE - 22.22))
                .build();

        Trajectory rightToBackdrop = drive.trajectoryBuilder(toAlign.end()) //
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 2 + 1.35, -TILE_SIZE - 24))
                .build();

        Trajectory backdropToPark = drive.trajectoryBuilder(new Pose2d(TILE_SIZE * 2, -TILE_SIZE - 22, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 2 + 2.4, -19, Math.toRadians(90)))
                .build();


        task = serial(
                conditional(() -> pos == AutoConstants.TSEPosition.LEFT, trajectory(leftToLine)),
                conditional(() -> pos == AutoConstants.TSEPosition.CENTER, trajectory(middleToLine)),
                conditional(() -> pos == AutoConstants.TSEPosition.RIGHT, trajectory(rightToLine)),
                execute(() -> {
                    intake.setPower(0.57);
                }),
                sleepms(1000),
                execute(() -> intake.setPower(0)),
                sleepms(600),
                conditional(() -> pos == AutoConstants.TSEPosition.LEFT, trajectory(leftAlignStack)),
                conditional(() -> pos == AutoConstants.TSEPosition.CENTER, trajectorySequence(middleAlignStack)),
                conditional(() -> pos == AutoConstants.TSEPosition.RIGHT, trajectorySequence(rightAlignStack)),

                trajectory(toAlign),
                parallel(
                        conditional(() -> pos == AutoConstants.TSEPosition.LEFT, trajectory(leftToBackdrop)),
                        conditional(() -> pos == AutoConstants.TSEPosition.CENTER, trajectory(middleToBackdrop)),
                        conditional(() -> pos == AutoConstants.TSEPosition.RIGHT, trajectory(rightToBackdrop)),
                        execute(() -> lift.setTargetPosition(950, 1))
                ),
                sleepms(200),
                execute(() -> {
                    claw.open();
                    lift.setTargetPosition(1110, 1);
                }),
                sleepms(380),
                parallel(
                        trajectory(backdropToPark),
                        serial(
                                sleepms(440),
                                execute(() -> lift.setTargetPosition(0, 1))
                        )
                ),
                sleepms(10000)
        );
    }
}
