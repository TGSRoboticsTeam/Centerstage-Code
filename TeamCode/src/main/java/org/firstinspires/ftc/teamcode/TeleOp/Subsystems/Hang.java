package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hang extends SubsystemBase {

    private final DcMotor lift;
    private final Servo liftArm;

    public enum ArmState{
        RAISED(.25),
        LOWERED(.44);

        private final double pos;

        ArmState(double pos) {
            this.pos = pos;
        }

        double getPos(){
            return pos;
        }
    }

    private ArmState armState = ArmState.LOWERED;

    public Hang(HardwareMap hardwareMap){
        lift = hardwareMap.get(DcMotor.class, "lift_mechanism");
        liftArm = hardwareMap.get(Servo.class, "lift_flipper");

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        liftArm.setPosition(.44);
    }

    public void moveArm(){
        if(armState == ArmState.LOWERED){
            armState = ArmState.RAISED;

            liftArm.setPosition(armState.getPos());
        }else{
            armState = ArmState.LOWERED;

            liftArm.setPosition(armState.getPos());
        }
    }

    public void setPower(double upPower, double downPower){
        lift.setPower(upPower - downPower);
    }
}
