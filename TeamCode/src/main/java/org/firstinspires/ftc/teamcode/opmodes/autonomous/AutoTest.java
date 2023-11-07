package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.ExecuteAction;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.dcMotor.get("a");

        waitForStart();

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(opModeIsActive() && !isStopRequested()) {
            motor.setPower(gamepad1.left_stick_y);
        }
//        Action test = new SequentialAction(
//                drive.actionBuilder(drive.pose)
//                        .splineTo(new Vector2d(0, 0), 0)
//                        .build()
//        );

//        Actions.runBlocking(test);
    }
}
