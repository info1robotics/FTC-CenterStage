package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Cane {
    DcMotor caneMotor;

    public Cane(HardwareMap hardwareMap) {
        this.caneMotor = hardwareMap.dcMotor.get("cane");
    }

    public void setPower(double power) {
        caneMotor.setPower(power);
    }
}
