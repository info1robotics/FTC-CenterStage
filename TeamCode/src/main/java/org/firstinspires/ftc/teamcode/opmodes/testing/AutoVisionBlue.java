package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutoBase;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineLeftBlue;
import org.firstinspires.ftc.teamcode.vision.TSEDetectionPipelineRightRed;

@Autonomous
public class AutoVisionBlue extends AutoBase {
    @Override
    public void onInit() {
        pos = Pos.BLUE_LEFT;
    }

    @Override
    public void onInitTick() {
        telemetry.addData("Position", ((TSEDetectionPipelineLeftBlue)pipeline).getAnalysis());
    }
}
