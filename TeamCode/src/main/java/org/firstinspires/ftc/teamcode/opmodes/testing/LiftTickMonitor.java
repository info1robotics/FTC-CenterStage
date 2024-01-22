package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
@Autonomous(group = "Debug")
public class LiftTickMonitor extends AutoBase {
    @Override
    public void onStartTick() {
        telemetry.addData("Lift Left", lift.liftLeft.getCurrentPosition());
        telemetry.addData("Lift Right", lift.liftRight.getCurrentPosition());

        lift.tick();
    }
}
