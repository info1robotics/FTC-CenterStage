package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Hook {
    DcMotor intake;
    public Hook(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("hook");
    }

    public void setPower(double power) {
        intake.setPower(power);
    }
}
