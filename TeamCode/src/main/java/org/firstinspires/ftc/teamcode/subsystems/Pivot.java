package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pivot {
    public static double PIVOT_DROP = 0.57;
    public static double PIVOT_COLLECT = 0;

    Servo servoLeft, servoRight; // when looking from the back
    public Pivot(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.servo.get("boxLeft");
        servoRight = hardwareMap.servo.get("boxRight");
        servoLeft.setDirection(Servo.Direction.REVERSE);
    }

    public void setPivotDrop() {
        servoLeft.setPosition(PIVOT_DROP);
        servoRight.setPosition(PIVOT_DROP);
    }

    public void setPivotCollect() {
        servoLeft.setPosition(PIVOT_COLLECT);
        servoRight.setPosition(PIVOT_COLLECT);
    }
}
