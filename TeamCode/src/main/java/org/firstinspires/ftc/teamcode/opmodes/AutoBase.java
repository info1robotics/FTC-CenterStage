package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public abstract class   AutoBase extends LinearOpMode {
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }

        return result;
    }
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
