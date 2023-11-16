package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class PivotIntake {
    public static double PIVOT_DROP = 0;
    public static double PIVOT_COLLECT = 0;

    public Servo servoLeft; // when looking from the intake
    public PivotIntake(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.servo.get("pivotIntakeLeft");
    }

    public void setDrop() {
        servoLeft.setPosition(PIVOT_DROP);
    }

    public void setCollect() {
        servoLeft.setPosition(PIVOT_COLLECT);
    }

    public void setPosLeft(double pos) {
        servoLeft.setPosition(pos);
    }
}
