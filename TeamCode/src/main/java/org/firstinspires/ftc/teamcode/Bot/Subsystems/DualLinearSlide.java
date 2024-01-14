package org.firstinspires.ftc.teamcode.Bot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.LimitedMotor;

public class DualLinearSlide extends SubsystemBase {
    private final LimitedMotor leftMotor, rightMotor;
    int maxHeight;

    public DualLinearSlide(HardwareMap hardwareMap, int maxHeight, String leftSlide, String rightSlide){
        leftMotor = new LimitedMotor(hardwareMap.get(DcMotorEx.class, leftSlide), 0, -maxHeight);
        rightMotor = new LimitedMotor(hardwareMap.get(DcMotorEx.class, rightSlide), 0, maxHeight);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        this.maxHeight = maxHeight;
    }

    public void setPower(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    /*public void moveToPosition(int position){
        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);
    }*/

    public double getAbsPosition(){
        return (Math.abs(leftMotor.getPosition()) + Math.abs(rightMotor.getPosition())) / 2;
    }

    public double getRightPosition(){
        return rightMotor.getPosition();
    }

    public double getLeftPosition(){
        return leftMotor.getPosition();
    }

    public boolean isBusy(){
        return leftMotor.isBusy() || rightMotor.isBusy();
    }
}
