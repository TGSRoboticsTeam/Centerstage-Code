package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit extends SubsystemBase {

    private final Servo deposit;
    private final Servo leftAligner, rightAligner;

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

    public void openAligner(){
        leftAligner.setPosition(.17);
        rightAligner.setPosition(.84);
    }

    public void closeAligner(){
        leftAligner.setPosition(.5);
        rightAligner.setPosition(.52);
    }
}
