package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;


@Config
@Autonomous(group = "Debug")
public class PivotIntakePositions extends AutoBase {
    public static double position = 0.00;

    @Override
    public void onStartTick() {
        pivotIntake.setPosLeft(position);
    }
}
