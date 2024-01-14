package org.firstinspires.ftc.teamcode.Bot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Bot.Commands.TriggerDeposit;
import org.firstinspires.ftc.teamcode.Bot.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Bot.Subsystems.DualLinearSlide;

public class CenterStageBot {
    public final DualLinearSlide linearSlides;
    public final Deposit deposit;

    public final TriggerDeposit depositIntake;
    public final TriggerDeposit depositOuttake;

    public CenterStageBot(HardwareMap hardwareMap){
        linearSlides = new DualLinearSlide(hardwareMap, 2500, "left_linear_slide", "right_linear_slide");
        deposit = new Deposit(hardwareMap, "claw");

        depositIntake = new TriggerDeposit(deposit, "intake");
        depositOuttake = new TriggerDeposit(deposit, "outtake");
    }
}
