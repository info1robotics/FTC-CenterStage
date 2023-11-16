package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous
public class AutoMecanumMeasure extends LinearOpMode {
    public static int WAIT_SECONDS = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain drive = new Drivetrain(this.hardwareMap);
        waitForStart();
        long time = System.currentTimeMillis();
        drive.fr.setPower(1);
        drive.fl.setPower(0.9958980369);
        drive.br.setPower(0.9795);
        drive.bl.setPower(0.9608);
        // bl 0.9354737456
        // br 0.9318864625
        // fl 0.9253396709
        // fr 0.9253396709
         while (opModeIsActive() && !isStopRequested() && time + WAIT_SECONDS * 1000L > System.currentTimeMillis()) {
            telemetry.addData("FR", drive.fr.getCurrentPosition());
            telemetry.addData("FL", drive.fl.getCurrentPosition());
            telemetry.addData("BR", drive.br.getCurrentPosition());
            telemetry.addData("BL", drive.bl.getCurrentPosition());
            telemetry.update();
        }
        drive.fr.setPower(0);
        drive.fl.setPower(0);
        drive.br.setPower(0);
        drive.bl.setPower(0);
    }
}
