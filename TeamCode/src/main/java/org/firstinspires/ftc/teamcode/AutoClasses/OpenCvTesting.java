package org.firstinspires.ftc.teamcode.AutoClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Open CV Testing", group = "Linear Opmode")
public class OpenCvTesting extends LinearOpMode
{
    OpenCvWebcam webcam = null;

    @Override
    public void runOpMode()
    {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Waiting on start");
            telemetry.update();
        }
    }
}
