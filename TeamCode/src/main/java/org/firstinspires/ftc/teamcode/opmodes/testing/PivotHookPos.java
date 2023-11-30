package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.subsystems.Hook;

@Autonomous
@Config
public class PivotHookPos extends LinearOpMode {
    public static double pos = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        Hook hook = new Hook(this.hardwareMap);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            hook.setPosition(pos);
        }
    }
}
