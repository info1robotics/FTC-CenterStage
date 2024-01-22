package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.subsystems.Cane;
import org.firstinspires.ftc.teamcode.subsystems.PivotIntake;

@Autonomous(group = "Prepare")
public class CalibratePivotIntake extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        PivotIntake pivotIntake = new PivotIntake(this.hardwareMap);
        waitForStart();
        pivotIntake.setPosLeft(0.22);
        while (!isStopRequested() && isStarted()) {

        }
    }
}
