package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {
    Servo clawLeft, clawRight;

    public static double RIGHT_CLOSED = 0.03;
    public static double LEFT_CLOSED = 0.004;

    public static double RIGHT_OPEN = 0.2;
    public static double LEFT_OPEN = 0.03;

    public static boolean clawLeftOpen = false;
    public static boolean clawRightOpen = false;

    public enum Type {
        LEFT, RIGHT
    }

    public Claw(HardwareMap hardwareMap) {
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawLeft.setDirection(Servo.Direction.REVERSE);
        clawRight = hardwareMap.servo.get("clawRight");
    }

    public void open() {
        clawLeft.setPosition(LEFT_OPEN);
        clawRight.setPosition(RIGHT_OPEN);
    }

    public void close() {
        clawLeft.setPosition(LEFT_CLOSED);
        clawRight.setPosition(RIGHT_CLOSED);
    }

    public void open(Type type) {
        if (type == Type.LEFT) {
            clawLeft.setPosition(LEFT_OPEN);
            clawLeftOpen = true;
        } else {
            clawRight.setPosition(RIGHT_OPEN);
            clawRightOpen = true;
        }
    }

    public void close(Type type) {
        if (type == Type.LEFT) {
            clawLeft.setPosition(LEFT_CLOSED);
            clawLeftOpen = false;
        } else {
            clawRight.setPosition(RIGHT_CLOSED);
            clawRightOpen = false;
        }
    }

    public void toggle(Type type) {
        if (type == Type.LEFT) {
            clawLeft.setPosition(clawLeftOpen ? LEFT_CLOSED : LEFT_OPEN);
            clawLeftOpen = !clawLeftOpen;
        } else {
            clawRight.setPosition(clawRightOpen ? RIGHT_CLOSED : RIGHT_OPEN);
            clawRightOpen = !clawRightOpen;
        }
    }
}
