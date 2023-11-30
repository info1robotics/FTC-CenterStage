package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.ROBOT_HEIGHT_HALF;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.TILE_SIZE;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.conditional;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.tasks.TaskBuilder.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.AutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineLeftBlue;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightBlue;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightRed;

import java.util.function.Supplier;

@Autonomous
public class AutoRightBlue extends AutoBase {
    Pose2d startPose = new Pose2d(-TILE_SIZE * 1.5 - 3.15, TILE_SIZE * 3 - ROBOT_HEIGHT_HALF, Math.toRadians(-90));

    AutoConstants.TSEPosition pos = AutoConstants.TSEPosition.LEFT;

    @Override
    public void onInitTick() {
        pos = ((TSEDetectionPipelineRightBlue) pipeline).getAnalysis();
        claw.close(Claw.Type.LEFT);
        telemetry.addData("pos", pos.toString());

    }

    @Override
    public void onInit() {
        startPos = Pos.BLUE_RIGHT;
        pivotIntake.setInit();
        drive.setPoseEstimate(startPose);

        Trajectory rightToLine = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 - 12.3, TILE_SIZE - 6.5, Math.toRadians(-90)))
                .build();

        Trajectory rightAlignBridge = drive.trajectoryBuilder(rightToLine.end())
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 + 7.1, TILE_SIZE / 2 - 3, Math.toRadians(0)))
                .build();

        Trajectory middleToLine = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 2, TILE_SIZE + 1.5, Math.toRadians(180)))
                .build();

        Trajectory middleAlignBridge = drive.trajectoryBuilder(middleToLine.end())
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 + 4.1, TILE_SIZE / 2 - 3, Math.toRadians(0)))
                .build();

        Trajectory leftToLine = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5, TILE_SIZE * 1.5 - 2, Math.toRadians(180)), Math.toRadians(-60))
                .build();

        Trajectory leftAlignBridge = drive.trajectoryBuilder(leftToLine.end())
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 1.5 + 7.1, TILE_SIZE / 2 - 3, Math.toRadians(0)))
                .build();

        Trajectory afterBridge = drive.trajectoryBuilder(middleAlignBridge.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.1, TILE_SIZE / 2 - 3))
                .build();

        Trajectory leftToBackdrop = drive.trajectoryBuilder(afterBridge.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 6.5, TILE_SIZE + 9.5))
                .build();

        Trajectory middleToBackdrop = drive.trajectoryBuilder(afterBridge.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 6.5, TILE_SIZE * 1 + 1.55))
                .build();

        Trajectory rightToBackdrop = drive.trajectoryBuilder(afterBridge.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 5.9, TILE_SIZE - 4))
                .build();

        Trajectory backdropToPark = drive.trajectoryBuilder(new Pose2d(TILE_SIZE * 1.5 + 3, TILE_SIZE * 1 + 4, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 2, TILE_SIZE * 3 - 20, Math.toRadians(-90)))
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
                conditional(() -> pos == AutoConstants.TSEPosition.LEFT, trajectory(leftAlignBridge)),
                conditional(() -> pos == AutoConstants.TSEPosition.CENTER, trajectory(middleAlignBridge)),
                conditional(() -> pos == AutoConstants.TSEPosition.RIGHT, trajectory(rightAlignBridge)),

                trajectory(afterBridge),
                parallel(
                        conditional(() -> pos == AutoConstants.TSEPosition.LEFT, trajectory(leftToBackdrop)),
                        conditional(() -> pos == AutoConstants.TSEPosition.CENTER, trajectory(middleToBackdrop)),
                        conditional(() -> pos == AutoConstants.TSEPosition.RIGHT, trajectory(rightToBackdrop)),
                        execute(() -> lift.setTargetPosition(950, 1))
                ),
                sleepms(200),
                execute(() -> {
                    claw.open(Claw.Type.LEFT);
                    lift.setTargetPosition(1110, 1);
                }),
                sleepms(380),
                parallel(
                        trajectory(backdropToPark),
                        serial(
                                sleepms(750),
                                execute(() -> lift.setTargetPosition(0, 1))
                        )
                ),
                sleepms(10000)
        );
    }
}
