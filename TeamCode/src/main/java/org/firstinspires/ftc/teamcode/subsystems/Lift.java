package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Pivot.PIVOT_TRANSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Teleop;

import java.text.DecimalFormat;

public class Lift {
    public DcMotor liftLeft, liftRight, liftLeft2;
    public static int UPPER = 1700;
    public static int LOWER = -15;

    public Lift(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.dcMotor.get("liftLeft");
        liftLeft2 = hardwareMap.dcMotor.get("liftLeft2");
        liftRight = hardwareMap.dcMotor.get("liftRight");

        MotorConfigurationType mct1 = liftLeft.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        liftLeft.setMotorType(mct1);

        MotorConfigurationType mct2 = liftLeft.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        liftRight.setMotorType(mct2);

        MotorConfigurationType mct3 = liftLeft.getMotorType().clone();
        mct3.setAchieveableMaxRPMFraction(1.0);
        liftRight.setMotorType(mct3);

        liftRight.setDirection(DcMotor.Direction.REVERSE);
        liftLeft2.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setTargetPosition(0);
        liftLeft2.setTargetPosition(0);
        liftRight.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders() {
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void tick() {
        int currentPosition = liftLeft.getCurrentPosition();
        if (currentPosition < -20) {
            resetEncoders();
        }
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
        liftLeft2.setTargetPosition(position);
        liftRight.setTargetPosition(position);


        liftLeft.setPower(power);
        liftLeft2.setPower(power);
        liftRight.setPower(power);
    }
}
