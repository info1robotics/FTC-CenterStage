package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@TeleOp(name = "Chassis")
public class Chassis extends LinearOpMode {
    GamepadEx gamepad1Ex, gamepad2Ex;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        Drivetrain drive = new Drivetrain(this.hardwareMap);
        Lift lift = new Lift(this.hardwareMap);


        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);



        waitForStart();

//        new Thread(() -> {
//            while (opModeIsActive()) {
//                drive.vectorMove(gamepad1.left_stick_x, -gamepad1.left_stick_y,
//                        (gamepad1.right_trigger - gamepad1.left_trigger),
//                        gamepad1.a ? 0.4 : 1.0);
//                gamepad1Ex.update();
//            }
//        }).start();


        while (opModeIsActive()) {
//            lift.setRawPower(-gamepad2.right_stick_y);
            telemetry.addData("Lift Left", lift.liftLeft.getCurrentPosition());
            telemetry.addData("Lift Right", lift.liftRight.getCurrentPosition());
            gamepad2Ex.update();
            telemetry.update();
        }

    }
}

