package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Lift {
    public DcMotor liftLeft, liftRight;
    public static int UPPER = 600;
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
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPower(double power) {
        if (power < -0.2) power = -0.2;
        setRawPower(power);
    }

    public void setRawPower(double power) {
        if (
                liftLeft.getMode() != DcMotor.RunMode.RUN_USING_ENCODER ||
                liftRight.getMode() != DcMotor.RunMode.RUN_USING_ENCODER
        ) {
            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

    public void setTargetPosition(int position) {
        liftLeft.setTargetPosition(position);
        liftRight.setTargetPosition(position);
        if (
                liftLeft.getMode() != DcMotor.RunMode.RUN_TO_POSITION ||
                liftRight.getMode() != DcMotor.RunMode.RUN_TO_POSITION
        ) {
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        liftLeft.setPower(1);
        liftRight.setPower(1);
    }
}
