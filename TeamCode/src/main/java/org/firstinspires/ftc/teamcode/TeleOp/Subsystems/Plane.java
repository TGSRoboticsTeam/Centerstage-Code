package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Plane extends SubsystemBase {

    private final Servo plane;

    public Plane(HardwareMap hardwareMap){
        plane = hardwareMap.get(Servo.class, "plane_launcher");

        plane.setPosition(0);
    }

    public void launchPlane(){
        plane.setPosition(1);
    }
}
