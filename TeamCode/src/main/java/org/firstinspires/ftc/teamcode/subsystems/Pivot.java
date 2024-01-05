package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Pivot {
    public static double PIVOT_DROP = 0.69;
    public static double PIVOT_TRANSITION = 0.07;
    public static double PIVOT_COLLECT = 0.0;

    public static Pivot instance;

    public Servo servoLeft, servoRight; // when looking from the intake
    public Pivot(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.servo.get("pivotLeft");
        servoRight = hardwareMap.servo.get("pivotRight");
        servoRight.setDirection(Servo.Direction.REVERSE);
        instance = this;
    }

    public void setDrop() {
        servoLeft.setPosition(PIVOT_DROP);
        servoRight.setPosition(PIVOT_DROP);
    }

    public void setCollect() {
        servoLeft.setPosition(PIVOT_COLLECT);
        servoRight.setPosition(PIVOT_COLLECT);
    }

    public void setTransition() {
        servoLeft.setPosition(PIVOT_TRANSITION);
        servoRight.setPosition(PIVOT_TRANSITION);
    }

    public void setPosLeft(double pos) {
        servoLeft.setPosition(pos);
    }

    public void setPosRight(double pos) {
        servoRight.setPosition(pos);
    }
}
