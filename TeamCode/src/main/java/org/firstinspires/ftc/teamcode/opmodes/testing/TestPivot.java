package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
@Config
public class TestPivot extends AutoBase {
    public static boolean test = false;
    @Override
    public void onStartTick() {
        telemetry.addData("test", pivot.servoLeft);
        telemetry.addData("tes1", pivot.servoRight);
        if (test) {
            pivot.setCollect();
        } else {
            pivot.setDrop();
        }
    }
}
