package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class PivotIntake {
    public static double PIVOT_MAX = 0.35;
    public static double PIVOT_TELEOP = 0.00;

    public Servo servoLeft; // when looking from the intake
    public PivotIntake(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.servo.get("pivotIntake");
    }

    public void setInit() {
        servoLeft.setPosition(PIVOT_MAX);
    }
    public void setNormal() {
        servoLeft.setPosition(PIVOT_TELEOP);
    }

    public void setPosLeft(double pos) {
        servoLeft.setPosition(pos);
    }
}
