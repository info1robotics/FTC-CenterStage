package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
@Config
public class AutoTest2 extends AutoBase {
    Servo servo3, servo2;
    public static double servoPos3 = 0.004, servoPos2 = 0.03;

    @Override
    public void onInit() {
        servo3 = hardwareMap.servo.get("3");
        servo3.setDirection(Servo.Direction.REVERSE);
        servo2 = hardwareMap.servo.get("2");
    }

    @Override
    public void onStartTick() {
        servo3.setPosition(servoPos3);
        servo2.setPosition(servoPos2);
    }
}