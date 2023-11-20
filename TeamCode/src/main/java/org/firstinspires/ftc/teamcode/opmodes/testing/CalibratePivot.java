package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
@Config
public class CalibratePivot extends AutoBase {
    public static double left = 0.0;
    public static double right = 0.0;
    @Override
    public void onStartTick() {
        pivot.servoRight.setPosition(right);
        pivot.servoLeft.setPosition(left);
        telemetry.addData("posleft", pivot.servoLeft.getPosition());
        telemetry.addData("posright", pivot.servoRight.getPosition());
    }
}
