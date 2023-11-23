package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
@Config
public class TestClaw extends AutoBase {
    public static boolean open = false;

    public static double clawLeft = 0;
    public static double clawRight = 0;

    @Override
    public void onStartTick() {
        telemetry.addData("test", claw.clawLeft.getPosition());
        telemetry.addData("tes1", claw.clawRight.getPosition());
        claw.clawLeft.setPosition(clawLeft);
        claw.clawRight.setPosition(clawRight);
    }
}
