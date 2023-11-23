package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightRed;

@Autonomous
public class AutoVisionRed extends AutoBase {
    @Override
    public void onInitTick() {
        telemetry.addData("Position", ((TSEDetectionPipelineRightRed )pipeline).getAnalysis());
    }
}
