package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Config
@Autonomous
public class CalibratePivot extends AutoBase {
//    public static double left = 0.0;
//    public static double right = 0.0;
    public static double pos = 0.0;
    @Override
    public void onStartTick() {
        pivot.servoRight.setPosition(pos);
        pivot.servoLeft.setPosition(pos);
        telemetry.addData("posLeft", pivot.servoLeft.getPosition());
        telemetry.addData("posRight", pivot.servoRight.getPosition());
    }
}
