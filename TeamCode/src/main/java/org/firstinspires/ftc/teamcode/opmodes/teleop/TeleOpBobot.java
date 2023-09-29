package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
public class TeleOpBobot extends LinearOpMode {
    public static final double OPEN_CLAW = .3;
    public static final double CLOSE_CLAW = 1;
    public static final int MAX_LIFT = 1000;
    public static final int MIN_LIFT = 0;
    DcMotor lift;
    Servo claw;
    boolean clawOpen = false;


    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(this.hardwareMap);
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        lift = hardwareMap.dcMotor.get("lift");
        claw = hardwareMap.servo.get("claw");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        claw.setPosition(CLOSE_CLAW);
        while (opModeIsActive()) {
            boolean lift_moved = false;

            drivetrain.vectorMove(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.left_trigger - gamepad1.right_trigger,
                    gamepad1.right_bumper ? 0.5 : 0.8
            );

            if (gamepadEx1.getButton("a")) {
                if (clawOpen) {
                    claw.setPosition(CLOSE_CLAW);
                } else {
                    claw.setPosition(OPEN_CLAW);
                }
                clawOpen = !clawOpen;
            }

            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(gamepad1.dpad_up ? 0.8 : -0.8);
//                if (lift.getCurrentPosition() > MAX_LIFT) lift.setPower(0);
                if (lift.getCurrentPosition() < MIN_LIFT) lift.setPower(0);
                lift_moved = true;
            }

            if (!lift_moved) {
                int targetHeight = lift.getCurrentPosition();

//                if (lift.getCurrentPosition() > MAX_LIFT) targetHeight = MAX_LIFT;
                if (lift.getCurrentPosition() < MIN_LIFT) targetHeight = MIN_LIFT;

                lift.setTargetPosition(targetHeight);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            }
            telemetry.update();
            gamepadEx1.update();
        }
    }
}
