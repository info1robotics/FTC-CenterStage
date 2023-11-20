package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
@Config
public class Rotate0 extends AutoBase {
    public static boolean test = false;
    Servo servoRight;

    @Override
    public void onStart() {
        servoRight = hardwareMap.servo.get("pivotRight");
    }

    @Override
    public void onStartTick() {
        servoRight.setPosition(0.0);
    }
}
