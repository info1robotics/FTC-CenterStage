package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous
@Config
public class A extends AutoBase {
    Servo pivotLeft;
    Servo pivotRight;

    public static double posLeft = 0;
    public static double posRight = 0;
    @Override
    public void onInit() {
        pivotLeft = hardwareMap.servo.get("pivotLeft");
        pivotLeft.setDirection(Servo.Direction.REVERSE);
        pivotRight = hardwareMap.servo.get("pivotRight");
        pivotRight.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void onStartTick() {
        pivotLeft.setPosition(posLeft);
        pivotRight.setPosition(posRight);
        telemetry.addData("a", System.currentTimeMillis());
        telemetry.addData("posright", posRight);
        telemetry.addData("posleft", posRight);

    }
}
