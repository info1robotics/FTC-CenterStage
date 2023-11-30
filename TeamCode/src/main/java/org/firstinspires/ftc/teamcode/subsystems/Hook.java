package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hook {
    public static double IDLE_POS = 0.6;
    public static double LIFT_POS = 1.0;

    public static int MAX_HEIGHT = 8800;
    public static int MIN_HEIGHT = 0;

    public DcMotor actuator;
    Servo gripper;
    public Hook(HardwareMap hardwareMap) {
        actuator = hardwareMap.dcMotor.get("hook");
        gripper = hardwareMap.servo.get("pivotHook");
    }

    public void setPower(double power) {
        actuator.setPower(power);

//        if (power > 0.1) {
//            actuator.setTargetPosition(MAX_HEIGHT);
//        } else if (power < -0.1) {
//            actuator.setTargetPosition(MIN_HEIGHT);
//        } else {
//            actuator.setTargetPosition(actuator.getCurrentPosition());
//        }
    }

    public void setPosition(double position) {
        gripper.setPosition(position);
    }

    public void setLift() {
        gripper.setPosition(LIFT_POS);
    }

    public void setIdle() {
        gripper.setPosition(IDLE_POS);
    }

    public void enableServo() {
        gripper.getController().pwmEnable();
    }

    public void disableServo() {
        gripper.getController().pwmDisable();
        gripper.getController().close();
    }
}
