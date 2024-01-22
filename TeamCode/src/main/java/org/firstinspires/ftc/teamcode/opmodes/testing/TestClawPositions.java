package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Config
@Autonomous(group = "Debug")
public class TestClawPositions extends AutoBase {
    public static double clawLeft = 0;
    public static double clawRight = 0;

    @Override
    public void onStartTick() {
        telemetry.addData("clawLeft", claw.clawLeft.getPosition());
        telemetry.addData("clawRight", claw.clawRight.getPosition());
        claw.clawLeft.setPosition(clawLeft);
        claw.clawRight.setPosition(clawRight);
    }
}

