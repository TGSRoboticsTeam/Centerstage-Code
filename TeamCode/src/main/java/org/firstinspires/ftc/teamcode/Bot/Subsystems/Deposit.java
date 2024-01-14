package org.firstinspires.ftc.teamcode.Bot.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
    private final Servo deposit;

    public enum DepositState {
        STORED(.61),
        ONE_PIXEL(.55),
        TWO_PIXEL(.34);

        private final double pos;
        DepositState(double pos) {
            this.pos = pos;
        }

        double getValue(){
            return pos;
        }
    }

    private DepositState depositState = DepositState.STORED;

    public Deposit(HardwareMap hardwareMap, String servoName){
        deposit = hardwareMap.get(Servo.class, servoName);
    }

    public void intake(){
        depositState = DepositState.TWO_PIXEL;

        setPosition(DepositState.TWO_PIXEL.getValue());
    }

    public void deposit(){
        if(depositState == DepositState.TWO_PIXEL){
            depositState = DepositState.ONE_PIXEL;

            setPosition(DepositState.ONE_PIXEL.getValue());
        }else{
            depositState = DepositState.STORED;

            setPosition(DepositState.STORED.getValue());
        }
    }

    public void setPosition(double position){
        deposit.setPosition(position);
    }

    public double getPosition(){
        return deposit.getPosition();
    }

}
