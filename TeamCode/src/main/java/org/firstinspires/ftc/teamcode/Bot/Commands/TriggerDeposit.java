package org.firstinspires.ftc.teamcode.Bot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Bot.Subsystems.Deposit;

public class TriggerDeposit extends CommandBase {

    private Deposit deposit;
    private String motion;

    public TriggerDeposit(Deposit deposit, String motion){
        this.deposit = deposit;
        this.motion = motion;

        addRequirements(deposit);
    }

    @Override
    public void initialize(){
        if(motion.equals("intake")){
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
