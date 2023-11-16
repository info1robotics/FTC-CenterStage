package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    DcMotor intake;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("intake");
    }

    public void setPower(double power) {
        intake.setPower(power);
    }
}
