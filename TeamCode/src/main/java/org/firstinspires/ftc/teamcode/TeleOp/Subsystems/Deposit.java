package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit extends SubsystemBase {

    private final Servo deposit;

    public enum DepositState {
        STORED(.25),
        ONE_PIXEL(.15),
        TWO_PIXEL(.04);

        private final double pos;

        DepositState(double pos) {
            this.pos = pos;
        }

        double getValue(){
            return pos;
        }
    }

    private DepositState depositState = DepositState.STORED;

    public Deposit(HardwareMap hardwareMap){
        deposit = hardwareMap.get(Servo.class, "claw");

        deposit.setPosition(depositState.getValue());
    }

    public void intake(){
        depositState = DepositState.TWO_PIXEL;

        deposit.setPosition(depositState.getValue());
    }

    public void outtake(){
        if(depositState == DepositState.TWO_PIXEL){
            depositState = DepositState.ONE_PIXEL;

            deposit.setPosition(depositState.getValue());
        }else{
            depositState = DepositState.STORED;

            deposit.setPosition(depositState.getValue());
        }
    }
}
