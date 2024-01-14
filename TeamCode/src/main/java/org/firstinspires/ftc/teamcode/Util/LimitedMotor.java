package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Creates a motor with a set min and max position. Use only with encoders.
 */
public class LimitedMotor {
    private final DcMotorEx motor;
    private final int min, max;
    private boolean negative = false;

    public LimitedMotor (DcMotorEx motor, int min, int max){
        this.motor = motor;
        this.min = min;
        this.max = max;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(max);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(0);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }

    public void setPower(double power){
        if(power > 0){
            if(negative){
                negative = false;
            }

            if(motor.getTargetPosition() != max){
                motor.setTargetPosition(max);
            }

            motor.setPower(power);
        }else{
            if(!negative){
                negative = true;
            }

            if(motor.getTargetPosition() != min){
                motor.setTargetPosition(min);
            }

            motor.setPower(-power);
        }
    }

    public void setTargetPosition(int pos)
    {
        motor.setTargetPosition(Range.clip(pos, min, max));
        motor.setPower(1);
    }

    public double getPower(){
        return motor.getPower();
    }

    public double getPosition(){
        return motor.getCurrentPosition();
    }

    public boolean isBusy(){
        return motor.isBusy();
    }
}
