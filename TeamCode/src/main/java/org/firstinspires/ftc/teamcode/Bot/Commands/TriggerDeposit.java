package org.firstinspires.ftc.teamcode.Bot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Bot.Subsystems.Deposit;

public class TriggerDeposit extends CommandBase {

    private final Deposit deposit;
    private final GamepadKeys.Button input;

    public TriggerDeposit(Deposit deposit, GamepadKeys.Button input){
        this.deposit = deposit;
        this.input = input;

        addRequirements(deposit);
    }

    @Override
    public void initialize(){
        if(input == GamepadKeys.Button.A){
            deposit.intake();
        }else{
            deposit.deposit();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
