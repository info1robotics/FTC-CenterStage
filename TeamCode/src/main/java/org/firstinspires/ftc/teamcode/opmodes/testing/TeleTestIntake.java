package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.PivotIntake;



@Config
@TeleOp
public class TeleTestIntake extends LinearOpMode {
    public static double pos = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        PivotIntake pivotIntake = new PivotIntake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
//            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//            if (gamepadEx1.getButtonDown("bumper_left")) {
//                claw.toggle(Claw.Type.LEFT);
//            } else if (gamepadEx1.getButtonDown("bumper_right")) {
//                claw.toggle(Claw.Type.RIGHT);
//            }
//
//            if (gamepadEx1.getButtonDown("a")) {
//                pivot.setCollect();
//            } else if (gamepadEx1.getButtonDown("y")) {
//                pivot.setDrop();
//            }
            telemetry.addData("a", pivotIntake.servoLeft.getPosition());
            pivotIntake.servoLeft.setPosition(pos);
            gamepadEx1.update();
            telemetry.update();
        }
    }
}
