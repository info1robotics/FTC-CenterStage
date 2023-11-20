package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
public class LiftTickMonitor extends AutoBase {
    @Override
    public void onStartTick() {
        telemetry.addData("Lift Left", lift.liftLeft.getCurrentPosition());
        telemetry.addData("Lift Right", lift.liftRight.getCurrentPosition());

        int currentPosition = lift.liftLeft.getCurrentPosition();
        if (currentPosition < 12) {
            pivot.setCollect();
        } else if (currentPosition > 12 && currentPosition < 290) {
            pivot.setTransition();
        } else if (currentPosition > 400) {
            pivot.setDrop();
        }
    }
}
