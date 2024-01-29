package org.firstinspires.ftc.teamcode.RandomClasses;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandVersions.CenterStageBot;
import org.firstinspires.ftc.teamcode.CommandVersions.Commands.MoveSlides;

@TeleOp(name = "Command Testing", group = "Testing")
@Disabled
public class CommandTesting extends CommandOpMode {

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
