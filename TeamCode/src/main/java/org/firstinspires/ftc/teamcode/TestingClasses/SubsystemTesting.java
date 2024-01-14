package org.firstinspires.ftc.teamcode.TestingClasses;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
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
import org.firstinspires.ftc.teamcode.Bot.Commands.TriggerDeposit;

@TeleOp(name = "Subsystem Tests", group = "Testing")

public class SubsystemTesting extends CommandOpMode {

    GamepadEx driver = new GamepadEx(gamepad1);
    GamepadEx placer = new GamepadEx(gamepad1);

    CenterStageBot robot;

    MoveSlides slides;

    @Override
    public void initialize(){
        driver = new GamepadEx(gamepad1);
        placer = new GamepadEx(gamepad1);

        robot = new CenterStageBot(hardwareMap);

        slides = new MoveSlides(robot.linearSlides, ()-> placer.getTrigger(
                GamepadKeys.Trigger.RIGHT_TRIGGER) - placer.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        placer.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(robot.depositIntake);
        placer.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(robot.depositOuttake);

        robot.linearSlides.setDefaultCommand(slides);
    }
}
