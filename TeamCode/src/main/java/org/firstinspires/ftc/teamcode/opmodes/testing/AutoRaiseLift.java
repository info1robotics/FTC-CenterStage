package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
public class AutoRaiseLift extends AutoBase {
    @Override
    public void onStart() throws InterruptedException {
        lift.setTargetPosition(950, 1);
    }
}
