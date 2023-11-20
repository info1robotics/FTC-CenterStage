package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Config
@Autonomous
public class AutoFindServo extends AutoBase {
    public static double servo0 = 0.5;
    Servo s0;

    @Override
    public void onStart() {
        s0 = hardwareMap.servo.get("0");
    }

    @Override
    public void onStartTick() {
        s0.setPosition(servo0);
    }
}
