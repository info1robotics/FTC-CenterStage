package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous
public class AutoParkRight extends AutoBase {
    @Override
    public void onStart() throws InterruptedException {
        Drivetrain drive = new Drivetrain(this.hardwareMap);
        drive.vectorMove(1, 0, 0, 0.8);
        Thread.sleep(900);
        drive.vectorMove(0, 0, 0, 0.8);
    }
}
