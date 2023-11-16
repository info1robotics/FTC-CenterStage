package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Autonomous
@Config
public class AutoTestClaw extends AutoBase {
    public static boolean open = false;
    Claw claw;
    @Override
    public void onInit() {
        claw = new Claw(hardwareMap);
    }

    @Override
    public void onStartTick() {
        if (open) claw.open();
        else claw.close();
    }
}
