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
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightRed;

@Autonomous
public class AutoRightRed extends AutoBase {
    Pose2d startPose = new Pose2d(TILE_SIZE * 0.5, -TILE_SIZE * 3 + ROBOT_HEIGHT_HALF, Math.toRadians(90));

    AutoConstants.TSEPosition pos;
//
//    // Since there is plenty of room for correction, we don't need to align ourselves and can rely on the PID correction
    Pose2d averageBackdrop = new Pose2d(TILE_SIZE * 2, -TILE_SIZE / 2 + 3, Math.toRadians(0));
//
    TrajectorySequence cycles;


    @Override
    public void onInitTick() {
        pos = ((TSEDetectionPipelineRightRed) pipeline).getAnalysis();
        claw.close(Claw.Type.LEFT);
        telemetry.addData("pos", pos.toString());

    }

    @Override
    public void onInit() {
        startPos = Pos.RED_RIGHT;
        pivotIntake.setInit();
        drive.setPoseEstimate(startPose);

        Trajectory leftToLine = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 0.5, -TILE_SIZE - 4, Math.toRadians(0)))
                .build();

        Trajectory leftToBackdrop = drive.trajectoryBuilder(leftToLine.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 13, -TILE_SIZE - 10.4))
                .build();

        Trajectory middleToLine = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 1, -TILE_SIZE - 1, Math.toRadians(0)))
                .build();

        Trajectory middleToBackdrop = drive.trajectoryBuilder(middleToLine.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 11.2, -TILE_SIZE * 1 - 17))
                .build();


        Trajectory rightToLine = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 1.5 - 2, -TILE_SIZE - 4, Math.toRadians(0)))
                .build();

        Trajectory rightToBackdrop = drive.trajectoryBuilder(rightToLine.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 11.2, -TILE_SIZE - 21.5))
                .build();

        Trajectory backdropToPark = drive.trajectoryBuilder(new Pose2d(TILE_SIZE * 1.5 + 3, -TILE_SIZE * 1 - 17, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 2, -TILE_SIZE * 3 + ROBOT_HEIGHT_HALF, Math.toRadians(90)))
                .build();

//        cycles = drive.trajectorySequenceBuilder(averageBackdrop)
//                .setReversed(true)
//                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 2 - 6.9, -TILE_SIZE / 2 + 3, Math.toRadians(0)))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    pivotIntake.setPosLeft(0.25);
//                    intake.setPower(-0.9);
//                })
//                .waitSeconds(.7)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    intake.setPower(0);
//                    pivotIntake.setInit();
//                })
//                .build();

        task = serial(
                conditional(() -> pos == AutoConstants.TSEPosition.LEFT, trajectory(leftToLine)),
                conditional(() -> pos == AutoConstants.TSEPosition.CENTER, trajectory(middleToLine)),
                conditional(() -> pos == AutoConstants.TSEPosition.RIGHT, trajectory(rightToLine)),
                execute(() -> {
                    if (pos == AutoConstants.TSEPosition.LEFT) intake.setPower(0.57);
                    else intake.setPower(0.5);
                }),
                sleepms(1000),
                execute(() -> intake.setPower(0)),
                parallel(
                        conditional(() -> pos == AutoConstants.TSEPosition.LEFT, trajectory(leftToBackdrop)),
                        conditional(() -> pos == AutoConstants.TSEPosition.CENTER, trajectory(middleToBackdrop)),
                        conditional(() -> pos == AutoConstants.TSEPosition.RIGHT, trajectory(rightToBackdrop)),
                        serial(
                                sleepms(100),
                                execute(() -> lift.setTargetPosition(820, 1))
                        )
                ),
                sleepms(200),
                execute(() -> {
                    claw.open(Claw.Type.LEFT);
                    lift.setTargetPosition(875, 1);
                }),
                sleepms(380),
                parallel(
                        trajectory(backdropToPark),
                        serial(
                                sleepms(200),
                                execute(() -> lift.setTargetPosition(0, 1))
                        )
                ),
                sleepms(10000)
        );
    }
}
