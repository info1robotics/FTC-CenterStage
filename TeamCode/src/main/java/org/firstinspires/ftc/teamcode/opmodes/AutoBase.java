package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.subsystems.PivotIntake;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public abstract class  AutoBase extends LinearOpMode {
    public OpenCvCamera camera;
    public TSEDetectionPipeline pipeline;
    public Pivot pivot;
    public Claw claw;
    public PivotIntake pivotIntake;
    public Intake intake;
    public Lift lift;
    static AutoBase instance = null;

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
//    public MecanumDrive drive;


    public void onInit() {}
    public void onInitTick() {}
    public void onStart() throws InterruptedException {}
    public void onStartTick() {}

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        instance = this;
//        enableVision();
        pivot = new Pivot(this.hardwareMap);
        claw = new Claw(this.hardwareMap);
        lift = new Lift(this.hardwareMap);
        intake = new Intake(this.hardwareMap);
        pivotIntake = new PivotIntake(this.hardwareMap);
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

    public void enableVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        telemetry.addData("camera ", cameraMonitorViewId);
        System.out.println(cameraMonitorViewId);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TSEDetectionPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }
}
