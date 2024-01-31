package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Deposit extends SubsystemBase {

    private final Servo deposit;
    private final Servo leftAligner, rightAligner;

    private boolean retracted = false;

    private ElapsedTime runtime = new ElapsedTime();
    private double timerTime = 0;
    private boolean timerSet = false;

    public enum DepositState {
        STORED(.95),
        ONE_PIXEL(.86),
        TWO_PIXEL(.7);

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

        leftAligner = hardwareMap.get(Servo.class, "left_aligner");
        rightAligner = hardwareMap.get(Servo.class, "right_aligner");

        deposit.setPosition(depositState.getValue());

        leftAligner.setPosition(.17);
        rightAligner.setPosition(.84);
    }

    public void intake(){
        depositState = DepositState.TWO_PIXEL;

        deposit.setPosition(depositState.getValue());

        /*leftAligner.setPosition(.18);
        rightAligner.setPosition(.84);*/

        setTimer(.5);

        retracted = false;
    }

    public void outtake(){
        if(depositState == DepositState.TWO_PIXEL){
            depositState = DepositState.ONE_PIXEL;

            deposit.setPosition(depositState.getValue());
            retracted = true;
        }else if(!retracted){
            depositState = DepositState.STORED;

            deposit.setPosition(depositState.getValue());
        }
    }

    public void openAligner(){
        leftAligner.setPosition(.18);
        rightAligner.setPosition(.84);
    }

    public void closeAligner(){
        leftAligner.setPosition(.485);
        rightAligner.setPosition(.52);
    }

    public DepositState getDepositState(){
        return depositState;
    }

    public void readyToRetract(){
        retracted = false;
    }

    public void setTimer(double time){
        timerTime = time;
        runtime.reset();

        timerSet = true;
    }

    public void checkTimer(){
        if(runtime.seconds() >= timerTime){
            timerSet = false;

            leftAligner.setPosition(.18);
            rightAligner.setPosition(.84);
        }
    }

    public boolean isTimerSet(){
        return timerSet;
    }
}
