package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

@Config
@Autonomous
public class TeleTestPivot extends AutoBase {
    public static double pivotLeft = 0;
    public static double pivotRight = 0;
    public static boolean update = false;
    boolean lastUpdate = true;




    @Override
    public void onStartTick() {
        telemetry.addData("Pivot Left", pivot.servoLeft.getPosition());
        telemetry.addData("Pivot Right", pivot.servoRight.getPosition());
        if (update != lastUpdate) {
            pivot.setPosLeft(pivotLeft);
            pivot.setPosRight(pivotRight);
        }
        lastUpdate = update;
    }
}
