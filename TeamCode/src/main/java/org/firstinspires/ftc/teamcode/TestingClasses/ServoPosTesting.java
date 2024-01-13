package org.firstinspires.ftc.teamcode.TestingClasses;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Servo Testing", group = "Testing")

public class ServoPosTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        ServoTester leftClawRotate = new ServoTester(hardwareMap, "left_claw_rotation", 1, 1);
        ServoTester2 rightClawRotate = new ServoTester2(hardwareMap, "right_claw_rotation", 1, 1);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Need this so that the code will stay initialized until you hit play on the phone
        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            if(gamepad1.dpad_up){
                leftClawRotate.setPositionUp();
                rightClawRotate.setPositionUp();
            }else if(gamepad1.dpad_down){
                leftClawRotate.setPositionDown();
                rightClawRotate.setPositionDown();
            }

            dashboardTelemetry.addData("Left Servo: ", leftClawRotate.getPosition());
            dashboardTelemetry.addData("Right Servo: ", rightClawRotate.getPosition());
            dashboardTelemetry.update();
        }
    }
}
