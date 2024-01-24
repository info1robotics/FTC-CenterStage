package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(group = "Debug")
public class TestPlane extends LinearOpMode {
    public static double position = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo plane = hardwareMap.servo.get("drone");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            plane.setPosition(position);
        }
    }
}
