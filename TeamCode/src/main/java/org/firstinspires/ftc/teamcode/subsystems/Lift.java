package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Teleop;

public class Lift {
    public DcMotor liftLeft, liftRight;
    public static int UPPER = 2330;
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
    }

    public void tick() {
        int currentPosition = liftLeft.getCurrentPosition();
        if (currentPosition < 12) {
            Pivot.instance.setCollect();
        } else if (currentPosition > 12 && currentPosition < 290) {
            Pivot.instance.setTransition();
        } else if (currentPosition > 400) {
            Pivot.instance.setDrop();
        }

//        if (AutoBase.getInstance() != null) {
//            AutoBase.getInstance().telemetry.addData("dada", liftLeft.getCurrentPosition());
//            AutoBase.getInstance().telemetry.addData("2dada", System.currentTimeMillis());
//        }
    }

    public void setPower(double power) {
        if (power < -0.6) power = -0.6;
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

    public void setTargetPosition(int position, double power) {
        liftLeft.setTargetPosition(position);
        liftRight.setTargetPosition(position);
        if (
                liftLeft.getMode() != DcMotor.RunMode.RUN_TO_POSITION ||
                liftRight.getMode() != DcMotor.RunMode.RUN_TO_POSITION
        ) {
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }
}
