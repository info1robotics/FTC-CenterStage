package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Cane;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.DroneLauncher;
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

        long planeLast = System.currentTimeMillis();
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        PivotIntake pivotIntake = new PivotIntake(hardwareMap);
        Cane cane = new Cane(hardwareMap);
        DroneLauncher drone=new DroneLauncher(hardwareMap);
        drone.setDefault();
        pivot.setCollect();

        claw.open();
        pivotIntake.setInit();
//        plane.setPosition(0);

        Drivetrain drive = new Drivetrain(hardwareMap);


        waitForStart();


        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
                    drive.vectorMove(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                            (gamepad1.right_trigger - gamepad1.left_trigger),
                            gamepad1.a ? 0.4 : 1.0);
                } else if (gamepad1.dpad_left) {
                    drive.vectorMove(-1, 0, 0, 0.5);
                } else if (gamepad1.dpad_right) {
                    drive.vectorMove(1, 0, 0, 0.5);
                }
                gamepadEx1.update();
            }
        }).start();

//        hook.setIdle();

        double lastRightStickY = 0;

        long lastTime = System.currentTimeMillis();
        while (opModeIsActive() && !isStopRequested()) {
            intake.setPower(-gamepad2.left_stick_y);
            double deltaTime = (System.currentTimeMillis() - lastTime) / 1000d;
            if (gamepad2.dpad_up) {
                double newPos = pivotIntake.servoRight.getPosition() + 2 * deltaTime;
                if (newPos > PivotIntake.PIVOT_MAX) newPos = PivotIntake.PIVOT_MAX;
                pivotIntake.servoRight.setPosition(newPos);
            } else if (gamepad2.dpad_down) {
                double newPos = pivotIntake.servoRight.getPosition() - 2 * deltaTime;
                if (newPos < PivotIntake.PIVOT_TELEOP) newPos = PivotIntake.PIVOT_TELEOP;
                pivotIntake.servoRight.setPosition(newPos);
            }

            lastTime = System.currentTimeMillis();
            telemetry.addData("voltage", getBatteryVoltage());

            cane.setPower(gamepad2.right_trigger - gamepad2.left_trigger);


            if (gamepadEx2.getButtonDown("bumper_left")) {
                claw.toggle(Claw.Type.LEFT);
            }
            if (gamepadEx2.getButtonDown("bumper_right")) {
                claw.toggle(Claw.Type.RIGHT);
            }


            if (gamepadEx2.getButtonDown("a")) {
                claw.open();
            } else if (gamepadEx2.getButtonDown("b")) {
                claw.close();
            }

            if (gamepad2.y && gamepad2.start) {
                drone.release();
            }

//            telemetry.addData("hook pos", hook.actuator.getCurrentPosition());

//            if (gamepadEx2.getButtonDown("dpad_right")) {
//                hook.setLift();
//            } else if (gamepadEx2.getButtonDown("dpad_left")) {
//                hook.setIdle();
//            }

            double rightStickY = -gamepad2.right_stick_y;
            telemetry.addData("Right Stick Y", -gamepad2.right_stick_y);

            if (rightStickY > 0.05 || rightStickY < -0.05) {
                lift.setPower(rightStickY);
            } else if (lastRightStickY > 0.05 || lastRightStickY < -0.05) {
                lift.setTargetPosition(lift.liftLeft.getCurrentPosition(), .4);
            }

            lastRightStickY = rightStickY;


            lift.tick();
            telemetry.addData("fdfsdfs", lift.liftLeft.getCurrentPosition());
            telemetry.addData("fdfsdfsfgdf", lift.liftRight.getCurrentPosition());
            telemetry.update();
            gamepadEx2.update();
        }

    }
}
