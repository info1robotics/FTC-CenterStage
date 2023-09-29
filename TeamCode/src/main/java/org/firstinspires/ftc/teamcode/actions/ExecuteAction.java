package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ExecuteAction implements Action {
    private Runnable runnable;

    public ExecuteAction(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        Action.super.preview(fieldOverlay);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        runnable.run();
        return false;
    }
}
