package org.firstinspires.ftc.teamcode.CommandVersions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.CommandVersions.Commands.TriggerDeposit;
import org.firstinspires.ftc.teamcode.CommandVersions.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.CommandVersions.Subsystems.DualLinearSlide;

public class CenterStageBot extends Robot {
    public final DualLinearSlide linearSlides;
    public final Deposit deposit;

    public TriggerDeposit depositIntake;
    public TriggerDeposit depositOuttake;

    public CenterStageBot(HardwareMap hardwareMap){
        linearSlides = new DualLinearSlide(hardwareMap, 2500, "left_linear_slide", "right_linear_slide");
        deposit = new Deposit(hardwareMap, "claw");

        depositIntake = new TriggerDeposit(deposit, "intake");
        depositOuttake = new TriggerDeposit(deposit, "outtake");
    }
}
