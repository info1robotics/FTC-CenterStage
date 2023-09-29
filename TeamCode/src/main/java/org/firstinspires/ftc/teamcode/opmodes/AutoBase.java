package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoBase extends LinearOpMode {
//    public MecanumDrive drive;


    public void onInit() {}
    public void onInitTick() {}
    public void onStart() {}
    public void onStartTick() {}

    @Override
    public void runOpMode() throws InterruptedException {
//        drive = new MecanumDrive(this.hardwareMap, new Pose2d(0, 0, 0));
        onInit();
        while (!isStarted() && !isStopRequested()) {
            onInitTick();
            telemetry.update();
        }
        onStart();
        while (opModeIsActive()) {
            onStartTick();
            telemetry.update();
        }
    }
}
