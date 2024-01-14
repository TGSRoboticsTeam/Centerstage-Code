package org.firstinspires.ftc.teamcode.Bot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Bot.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Bot.Subsystems.DualLinearSlide;

public class CenterStageBot {
    private final DualLinearSlide linearSlides;
    private final Deposit deposit;

    public CenterStageBot(HardwareMap hardwareMap){
        linearSlides = new DualLinearSlide(hardwareMap, 2500, "left_linear_slide", "right_linear_slide");
        deposit = new Deposit(hardwareMap, "claw");
    }
}
