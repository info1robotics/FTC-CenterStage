package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.common.AutoConstants.ROBOT_HEIGHT_HALF;
import static org.firstinspires.ftc.teamcode.common.AutoConstants.TILE_SIZE;

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

    TrajectorySequence left, middle, right, toFollow;

    // Since there is plenty of room for correction, we don't need to align ourselves and can rely on the PID correction
    Pose2d averageBackdrop = new Pose2d(TILE_SIZE * 2, -TILE_SIZE / 2 + 3, Math.toRadians(0));

    TrajectorySequence cycles;

    @Override
    public void onInitTick() {
        AutoConstants.TSEPosition pos = ((TSEDetectionPipelineRightRed)pipeline).getAnalysis();

        if (pos == AutoConstants.TSEPosition.LEFT) {
            toFollow = left;
        } else if (pos == AutoConstants.TSEPosition.CENTER) {
            toFollow = middle;
        } else if (pos == AutoConstants.TSEPosition.RIGHT) {
            toFollow = right;
        }

        pivot.setTransition();
        claw.close(Claw.Type.LEFT);

    }

    @Override
    public void onInit() {
        pivotIntake.setInit();
        drive.setPoseEstimate(startPose);

        left = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 0.5, -TILE_SIZE - 4, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 2, -TILE_SIZE - 8.9))
                .build();

        middle = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 1, -TILE_SIZE - 1, Math.toRadians(0)))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.setPower(0.5);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.setPower(0);
                    lift.setTargetPosition(250, 0.5);
                })
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 2, -TILE_SIZE * 1.5))
                 .build();

        right = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(TILE_SIZE * 1.5, -TILE_SIZE - 4, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(TILE_SIZE * 2, -TILE_SIZE - 20))
                .build();

        cycles = drive.trajectorySequenceBuilder(averageBackdrop)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-TILE_SIZE * 2 - 6.9, -TILE_SIZE / 2 + 3, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    pivotIntake.setPosLeft(0.25);
                    intake.setPower(-0.9);
                })
                .waitSeconds(.7)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.setPower(0);
                    pivotIntake.setInit();
                })
                .build();

    }

    @Override
    public void onStart() throws InterruptedException {
        drive.followTrajectorySequence(middle);
//        drive.followTrajectorySequence(cycles);
    }
}
