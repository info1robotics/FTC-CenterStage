package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
public class AutoTest2 extends AutoBase {
    Servo servo1, servo2;
    public static double a = 0.1;

    @Override
    public void onInit() {
        servo1 = hardwareMap.servo.get("1");
//        servo2 = hardwareMap.servo.get("2");
//        servo1.setDirection(Servo.Direction.REVERSE);
        servo1.setPosition(a);
//        servo2.setPosition(0);
    }

    @Override
    public void onStart() {
//        servo1.setPosition(1);
//        servo2.setPosition(1);
    }
}
