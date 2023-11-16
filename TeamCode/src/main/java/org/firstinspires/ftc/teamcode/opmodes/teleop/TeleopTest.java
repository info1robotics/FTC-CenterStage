package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
public class TeleopTest extends LinearOpMode {
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        Drivetrain drive = new Drivetrain(hardwareMap);
        MecanumDrive rrDrive = new MecanumDrive(this.hardwareMap, new Pose2d(0,0,0));

//        gamepad1.setLedColor(0, 255, 0, Integer.MAX_VALUE);

        waitForStart();


//        new Thread(() -> {
        while (opModeIsActive() && !isStopRequested()) {
            drive.vectorMove(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    (gamepad1.right_trigger - gamepad1.left_trigger),
                    1);
            telemetry.update();
            gamepadEx1.update();
//            if (gamepad1.dpad_up) {
//                drive.vectorMove(0, 1, 0, gamepad1.right_trigger);
//            } else if (gamepad1.dpad_down) {
//                drive.vectorMove(0, -1, 0, gamepad1.right_trigger);
//            } else if (gamepad1.dpad_left) {
//                drive.vectorMove(-1, 0, 0, gamepad1.right_trigger);
//            } else if (gamepad1.dpad_right) {
//                drive.vectorMove(1, 0, 0, gamepad1.right_trigger);
//            } else {
//                drive.vectorMove(0, 0, 0, 0);
//            }
//            drive.bl.setPower(gamepad1.dpad_left ? 1 : 0);
//            drive.br.setPower(gamepad1.dpad_up ? 1 : 0);
//            drive.fl.setPower(gamepad1.dpad_right ? 1 : 0);
//            drive.fr.setPower(gamepad1.dpad_down ? 1 : 0);
        }
//        }).start();

//        while (opModeIsActive()) {
//            if (gamepadEx2.getButtonDown("y")) {
//                pivot.setPivotDrop();
//            }
//            if (gamepadEx2.getButtonDown("b")) {
//                pivot.setPivotCollect();
//            }
//            if (gamepadEx2.getButtonDown("x")) {
//                trapdoor.toggleTrapdoor();
//            }
//
//            intake.setPower(-gamepad1.right_stick_y);
//
//
//            telemetry.addData("Y Position Joystick", gamepad1.right_stick_y);
//            telemetry.addData("X Position Joystick", gamepad1.right_stick_x);
//            telemetry.addData("Lift Left", lift.liftLeft.getCurrentPosition());
//            telemetry.addData("Lift Right", lift.liftRight.getCurrentPosition());
//            telemetry.addData("Voltage", getBatteryVoltage());
//            if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
//                double realY = -gamepad2.right_stick_y;
//                lift.setPower(realY);
//            } else {
//                lift.setTargetPosition(lift.liftRight.getCurrentPosition());
//            }
//
//            telemetry.addData("Touch Position X", gamepad2.touchpad_finger_1_x);
//            telemetry.addData("Touch Position Y", gamepad2.touchpad_finger_1_y);
//            telemetry.addData("Touch", gamepad2.touchpad);
//
//            drive.bl.setPower(gamepad1.dpad_left ? 1 : 0);
//            drive.br.setPower(gamepad1.dpad_up ? 1 : 0);
//            drive.fl.setPower(gamepad1.dpad_right ? 1 : 0);
//            drive.fr.setPower(gamepad1.dpad_down ? 1 : 0);
//
//            gamepadEx1.update();
//            gamepadEx2.update();
//            telemetry.update();
//        }
    }
}
