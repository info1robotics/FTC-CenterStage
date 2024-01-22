package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.subsystems.Cane;

@Autonomous(group = "Prepare")
public class RaiseCane extends AutoBase {
    @Override
    public void onStart() throws InterruptedException {
        Cane cane = new Cane(hardwareMap);
        cane.setPower(0.5);
    }
}
