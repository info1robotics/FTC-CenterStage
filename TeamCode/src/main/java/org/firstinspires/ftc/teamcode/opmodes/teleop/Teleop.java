package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.PivotIntake;

@TeleOp
public class Teleop extends LinearOpMode {
    public static Teleop instance;

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
        instance = this;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        PivotIntake pivotIntake = new PivotIntake(hardwareMap);

        claw.open();
        pivotIntake.setNormal();

        Drivetrain drive = new Drivetrain(hardwareMap);


        waitForStart();


        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                drive.vectorMove(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                        (gamepad1.right_trigger - gamepad1.left_trigger),
                        1.0);
                gamepadEx1.update();
            }
        }).start();

        long lastTime = System.currentTimeMillis();
        while (opModeIsActive() && !isStopRequested()) {
            intake.setPower(-gamepad2.left_stick_y);
            double deltaTime = (System.currentTimeMillis() - lastTime) / 1000d;
            if (gamepad2.dpad_up) {
                double newPos = pivotIntake.servoLeft.getPosition() + 2 * deltaTime;
                if (newPos > PivotIntake.PIVOT_MAX) newPos = PivotIntake.PIVOT_MAX;
                pivotIntake.servoLeft.setPosition(newPos);
            } else if (gamepad2.dpad_down) {
                double newPos = pivotIntake.servoLeft.getPosition() - 2 * deltaTime;
                if (newPos < PivotIntake.PIVOT_TELEOP) newPos = PivotIntake.PIVOT_TELEOP;
                pivotIntake.servoLeft.setPosition(newPos);
            }
            lastTime = System.currentTimeMillis();
            telemetry.addData("voltage", getBatteryVoltage());


            if (gamepadEx2.getButtonDown("bumper_left")) {
                claw.toggle(Claw.Type.LEFT);
            }
            if (gamepadEx2.getButtonDown("bumper_right")) {
                claw.toggle(Claw.Type.RIGHT);
            }

            if (gamepadEx2.getButtonDown("a")) {
                 claw.open();
            }

            // lift logic
            int currentPosition = lift.liftLeft.getCurrentPosition();
            double rightStickY = -gamepad2.right_stick_y;
            telemetry.addData("Right Stick Y", -gamepad2.right_stick_y);
            if (currentPosition < 12) {
                pivot.setCollect();
            } else if (currentPosition > 12 && currentPosition < 290) {
                pivot.setTransition();
            } else if (currentPosition > 400) {
                pivot.setDrop();
            }
            int a = 0;
            if (rightStickY > 0.1 || rightStickY < -0.1) {
                if (lift.liftLeft.getCurrentPosition() < Lift.LOWER && rightStickY < -0.1) {
                    a = 7;
                    lift.setTargetPosition(Lift.LOWER, 1);
                } else if (lift.liftRight.getCurrentPosition() > Lift.UPPER && rightStickY > 0.1) {
                    a = 8;
                    lift.setTargetPosition(Lift.UPPER, 1);
                }
                lift.setPower(-gamepad2.right_stick_y);
                a = 1;

            } else {
                if (gamepad2.right_stick_button) {
                    a = 5;
                    lift.setTargetPosition(-5, .5);
                } else {
                    a = 4;
                    lift.setTargetPosition(lift.liftLeft.getCurrentPosition(), .9);
                }
                a = 2;
            }
            telemetry.addData("test", a);


            telemetry.update();
            gamepadEx2.update();
        }

    }
}
