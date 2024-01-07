package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class PivotIntake {
    public static double PIVOT_MAX = 0.36;
    public static double PIVOT_TELEOP = 0.034;

    public Servo servoRight; // when looking from the intake
    public PivotIntake(HardwareMap hardwareMap) {
        servoRight = hardwareMap.servo.get("pivotIntake");
        servoRight.setDirection(Servo.Direction.REVERSE);
    }

    public void setInit() {
        servoRight.setPosition(PIVOT_MAX);
    }
    public void setNormal() {
        servoRight.setPosition(PIVOT_TELEOP);
    }

    public void setPosLeft(double pos) {
        servoRight.setPosition(pos);
    }
}
