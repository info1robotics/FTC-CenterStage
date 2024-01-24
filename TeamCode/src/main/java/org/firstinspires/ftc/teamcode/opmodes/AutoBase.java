package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.AutoConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.PivotIntake;
import org.firstinspires.ftc.teamcode.tasks.Task;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineLeftBlue;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineLeftRed;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightBlue;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class AutoBase extends LinearOpMode {
    public static State state = State.DEFAULT;
    static AutoBase instance = null;
    private static AutoConstants.TSEPosition detectedZone = AutoConstants.TSEPosition.RIGHT;
    public OpenCvCamera camera;
    public OpenCvPipeline pipeline;
    public Pivot pivot;
    public Claw claw;
    public PivotIntake pivotIntake;
    public Intake intake;
    public Lift lift;
    public SampleMecanumDrive drive;
    public Pos startPos;
    public Task task;

    public boolean full = true;

    public static AutoConstants.TSEPosition getDetectedZone() {
        return detectedZone;
    }

    public static void setDetectedZone(AutoConstants.TSEPosition zone) {
        if (state == State.START) {
            return;
        }
        detectedZone = zone;
    }

    public static AutoBase getInstance() {
        return instance;
    }

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

    public void onInit() {
    }

    public void onInitTick() {
    }
//    public MecanumDrive drive;

    public void onStart() throws InterruptedException {
    }

    public void onStartTick() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        instance = this;
        pivot = new Pivot(this.hardwareMap);
        lift = new Lift(this.hardwareMap);
        claw = new Claw(this.hardwareMap);
        intake = new Intake(this.hardwareMap);
        pivotIntake = new PivotIntake(this.hardwareMap);
        drive = new SampleMecanumDrive(this.hardwareMap);

        pivotIntake.setInit();
        Servo plane = hardwareMap.servo.get("drone");
        plane.setPosition(0.34);

        onInit();
        state = State.INIT;
        enableVision();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("A/X for Full");
            telemetry.addLine("B/O for Detection");
            if (gamepad1.a) {
                full = true;
            }
            if (gamepad1.b) {
                full = false;
            }
            if (full) {
                telemetry.addLine("Running Full");
            } else {
                telemetry.addLine("Detection Only");
            }
            drive.update();
            onInitTick();
            telemetry.update();
        }

        camera.closeCameraDeviceAsync(() -> {});
        onStart();


//        pivotIntake.setPosLeft(0.55);
        state = State.START;
        if (task != null) task.start(this);

        while (opModeIsActive() && !isStopRequested()) {
            if (task != null) task.tick();
            lift.tick();
            onStartTick();
            telemetry.update();
        }

    }

    public void enableVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData("camera ", cameraMonitorViewId);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        if (startPos == Pos.BLUE_LEFT) {
            pipeline = new TSEDetectionPipelineLeftBlue();
        } else if (startPos == Pos.RED_RIGHT) {
            pipeline = new TSEDetectionPipelineRightRed();
        } else if (startPos == Pos.BLUE_RIGHT) {
            pipeline = new TSEDetectionPipelineRightBlue();
        } else if (startPos == Pos.RED_LEFT) {
            pipeline = new TSEDetectionPipelineLeftRed();
        }
        camera.setPipeline(pipeline);

        try {

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        } catch (Exception e) {

        }

    }

    public enum Pos {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_RIGHT,
        RED_LEFT
    }

    public enum State {
        DEFAULT,
        INIT,
        START
    }
}
