package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
@Config
public class AutoTest3 extends AutoBase {
    Servo servo;
    public static double servoPos = 0;

    @Override
    public void onInit() {
        servo = hardwareMap.servo.get("trapdoor");
    }

    @Override
    public void onStartTick() {
        servo.setPosition(servoPos);
    }
}
