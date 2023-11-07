package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Trapdoor {
    public static double TRAPDOOR_CLOSED = 0.92;
    public static double TRAPDOOR_OPEN = 1;

    Servo trapdoor;
    boolean open = false;
    public Trapdoor(HardwareMap hardwareMap) {
        trapdoor = hardwareMap.servo.get("trapdoor");
    }

    public void setTrapdoorClosed() {
        open = false;
        trapdoor.setPosition(TRAPDOOR_CLOSED);
    }

    public void setTrapdoorOpen() {
        open = true;
        trapdoor.setPosition(TRAPDOOR_OPEN);
    }

    public void toggleTrapdoor() {
        if (open) {
            setTrapdoorClosed();
        } else {
            setTrapdoorOpen();
        }
    }
}
