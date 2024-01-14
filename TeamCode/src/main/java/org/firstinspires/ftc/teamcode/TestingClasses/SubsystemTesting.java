package org.firstinspires.ftc.teamcode.TestingClasses;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Bot.CenterStageBot;
import org.firstinspires.ftc.teamcode.Bot.Commands.MoveSlides;

@TeleOp(name = "Subsystem Tests", group = "Testing")

public class SubsystemTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx placer = new GamepadEx(gamepad1);

        CenterStageBot robot = new CenterStageBot(hardwareMap);

        MoveSlides slides = new MoveSlides(robot.linearSlides, ()-> driver.getTrigger(
                GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Need this so that the code will stay initialized until you hit play on the phone
        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            robot.linearSlides.setDefaultCommand(slides);

            dashboardTelemetry.update();
        }
    }
}
