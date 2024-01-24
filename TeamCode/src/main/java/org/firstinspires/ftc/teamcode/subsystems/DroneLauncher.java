package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DroneLauncher {
    public Servo drone;
    public double armedPosition=1.0;
    public double releasePosition=0.5;
    public DroneLauncher(HardwareMap hardwareMap){
        drone=hardwareMap.servo.get("drone");

    }
    public void release ()
    {
            drone.setPosition(releasePosition);
    }
    public void setDefault()
    {
        drone.setPosition(armedPosition);
    }

}
