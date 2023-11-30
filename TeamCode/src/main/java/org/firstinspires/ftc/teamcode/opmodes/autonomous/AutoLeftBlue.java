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
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightRed;

import java.util.function.Supplier;

@Autonomous
public class AutoLeftBlue extends AutoBase {
    Pose2d startPose = new Pose2d(TILE_SIZE * 0.5  + 3.15, TILE_SIZE * 3 - ROBOT_HEIGHT_HALF, AutoConstants.HEADING_TO_RED);
    Vector2d alignPos = new Vector2d(TILE_SIZE * 1.5, TILE_SIZE / 2 - 3);

    AutoConstants.TSEPosition pos = AutoConstants.TSEPosition.CENTER;

    @Override
    public void onInitTick() {
        pos = ((TSEDetectionPipelineLeftBlue) pipeline).getAnalysis();
        claw.close(Claw.Type.LEFT);
        telemetry.addData("pos", pos.toString());
        telemetry.addData("pos2", pos.toString());

    }

    @Override
    public void onInit() {
        startPos = Pos.BLUE_LEFT;
        pivotIntake.setInit();
        drive.setPoseEstimate(startPose);

        Trajectory rightToLine = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 0.5 - 2, TILE_SIZE + 4, AutoConstants.HEADING_TO_BACKDROP))
                .build();

        Trajectory rightToBackdrop = drive.trajectoryBuilder(rightToLine.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 9.15, TILE_SIZE + 1.5))
                .build();

        Trajectory rightToAlign = drive.trajectoryBuilder(rightToBackdrop.end())
                .lineToConstantHeading(alignPos)
                .build();

        Trajectory middleToLine = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 1, TILE_SIZE - 1, AutoConstants.HEADING_TO_BACKDROP))
                .build();

        Trajectory middleToBackdrop = drive.trajectoryBuilder(middleToLine.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 9, TILE_SIZE * 1 + 7))
                .build();

        Trajectory middleToAlign = drive.trajectoryBuilder(middleToBackdrop.end())
                .lineToConstantHeading(alignPos)
                .build();

        Trajectory leftToLine = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 1.5 - 4.7, TILE_SIZE + 4, AutoConstants.HEADING_TO_BACKDROP))
                .build();

        Trajectory leftToBackdrop = drive.trajectoryBuilder(leftToLine.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 9, TILE_SIZE + 15.5))
                .build();

        Trajectory leftToAlign = drive.trajectoryBuilder(leftToBackdrop.end())
                .lineToConstantHeading(alignPos)
                .build();

        Trajectory toStack1 = drive.trajectoryBuilder(leftToAlign.end())
                .lineToConstantHeading(new Vector2d(-TILE_SIZE * 2.5 - 1.25, TILE_SIZE / 2 + 2.4))
                .build();

        Trajectory toAlign1 = drive.trajectoryBuilder(toStack1.end())
                .lineToLinearHeading(toStack1.start())
                .build();

        Trajectory toBackdrop1 = drive.trajectoryBuilder(toAlign1.end())
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 1.5 + 9, TILE_SIZE + 14.4))
                .build();

        Trajectory park = drive.trajectoryBuilder(toBackdrop1.end())
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 1.9, TILE_SIZE * 2.5, AutoConstants.HEADING_TO_RED))
                .build();





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
                    lift.setTargetPosition(1085, 1);
                }),
                parallel(
                        conditional(() -> pos == AutoConstants.TSEPosition.LEFT, trajectory(leftToAlign)),
                        conditional(() -> pos == AutoConstants.TSEPosition.CENTER, trajectory(middleToAlign)),
                        conditional(() -> pos == AutoConstants.TSEPosition.RIGHT, trajectory(rightToAlign)),
                        serial(
                                sleepms(400),
                                execute(() -> lift.setTargetPosition(0, 1)),
                                execute(() -> claw.open())
                        )
                ),
                trajectory(toStack1),
                execute(() -> intake.setPower(-0.87)),
                sleepms(3000),
                execute(() -> claw.close()),
                sleepms(150),
                execute(() -> intake.setPower(1)),
                sleepms(50),
                parallel(
                        trajectory(toAlign1),
                        serial(
                                sleepms(1040),
                                execute(() -> intake.setPower(0))
                        )
                ),
                sleepms(100),
                parallel(
                        trajectory(toBackdrop1),
                        serial(
                                sleepms(100),
                                execute(() -> lift.setTargetPosition(1020, 1))
                        )
                ),
                sleepms(400),
                execute(() -> {
                    claw.open();
                    lift.setTargetPosition(1300, 1);
                }),
                sleepms(500),
                parallel(
                        trajectory(park),
                        serial(
                                sleepms(450),
                                execute(() -> lift.setTargetPosition(0, 1))
                        )
                ),
                sleepms(10000)
        );
    }
}
