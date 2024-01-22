package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
@Disabled
public class AutoRaiseLift extends AutoBase {
    @Override
    public void onStart() throws InterruptedException {
        lift.setTargetPosition(950, 1);
    }
}
