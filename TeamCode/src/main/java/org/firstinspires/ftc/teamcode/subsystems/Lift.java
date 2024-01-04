package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Pivot.PIVOT_TRANSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Teleop;

import java.text.DecimalFormat;

public class Lift {
    public DcMotor liftLeft, liftRight;
    public static int UPPER = 1700;
    public static int LOWER = 0;

    public Lift(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftRight = hardwareMap.dcMotor.get("liftRight");

        MotorConfigurationType mct1 = liftLeft.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        liftLeft.setMotorType(mct1);

        MotorConfigurationType mct2 = liftLeft.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        liftRight.setMotorType(mct2);

        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void tick() {
        int currentPosition = liftLeft.getCurrentPosition();
        if (currentPosition < 12) {
            Pivot.instance.setCollect();
        } else if (currentPosition > 15 && currentPosition < 120) {
            double servoPosIntermediary = (PIVOT_TRANSITION / (140 + 5 - currentPosition)) * 25;
//            double servoPosIntermediary = PIVOT_TRANSITION / (140 - currentPosition);
            Pivot.instance.setPosRight(servoPosIntermediary);
            Pivot.instance.setPosLeft(servoPosIntermediary);
//            Teleop.instance.telemetry.addData("servoPos", new DecimalFormat("#.0000").format(servoPosIntermediary));
        } else if (currentPosition > 170) {
            Pivot.instance.setDrop();
        }

    }

    public void setPower(double power) {
        if (power < -0.7) power = -0.7;
        if (power < 0) {
            setTargetPosition(LOWER, -power);
        } else {
            setTargetPosition(UPPER, power);
        }
    }

    public void setTargetPosition(int position, double power) {
        liftLeft.setTargetPosition(position);
        liftRight.setTargetPosition(position);


        liftLeft.setPower(power);
        liftRight.setPower(power);
    }
}
