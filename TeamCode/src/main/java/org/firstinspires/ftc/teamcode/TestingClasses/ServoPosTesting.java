package org.firstinspires.ftc.teamcode.TestingClasses;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TestingClasses.ConfigClasses.ServoTester;
import org.firstinspires.ftc.teamcode.TestingClasses.ConfigClasses.ServoTester2;

@TeleOp(name = "Servo Testing", group = "Testing")

public class ServoPosTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        ServoTester servo1 = new ServoTester(hardwareMap, "claw", 1, 1);
        ServoTester2 servo2 = new ServoTester2(hardwareMap, "right_claw_rotation", 1, 1);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Need this so that the code will stay initialized until you hit play on the phone
        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                servo1.setPositionUp();
                //servo2.setPositionUp();
            }else if(gamepad1.dpad_down){
                servo1.setPositionDown();
                //servo2.setPositionDown();
            }

            dashboardTelemetry.addData("Servo One", servo1.getPosition());
            //dashboardTelemetry.addData("Servo Two", servo2.getPosition());
            dashboardTelemetry.update();
        }
    }
}
