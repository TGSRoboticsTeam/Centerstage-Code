package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearSlides extends SubsystemBase {

    private final DcMotor leftSlide, rightSlide;
    private final Servo leftArm, rightArm;

    private final int maxHeight = 2500;

    private final double armRotationPos = -750;

    private final double leftArmUpPos = .68;
    private final double leftArmDownPos = .91;

    private final double rightArmUpPos = .835;
    private final double rightArmDownPos = .61;

    public LinearSlides(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        leftArm = hardwareMap.get(Servo.class, "left_claw_rotation");
        rightArm = hardwareMap.get(Servo.class, "right_claw_rotation");

        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArm.setPosition(leftArmDownPos);
        rightArm.setPosition(rightArmDownPos);
    }

    public void setPower(double upPower, double downPower){
        if (leftSlide.getCurrentPosition() < 0) {
            leftSlide.setPower(upPower);
        }else if (leftSlide.getCurrentPosition() > maxHeight) {
            leftSlide.setPower(-downPower);
        }else{
            leftSlide.setPower(upPower - downPower);
        }

        if (rightSlide.getCurrentPosition() < 0) {
            rightSlide.setPower(upPower);
        }else if (rightSlide.getCurrentPosition() > maxHeight) {
            rightSlide.setPower(-downPower);
        }else{
            rightSlide.setPower(upPower - downPower);
        }

        if (rightSlide.getCurrentPosition() > -armRotationPos){
            leftArm.setPosition(leftArmUpPos);
            rightArm.setPosition(rightArmUpPos);
        }else{
            leftArm.setPosition(leftArmDownPos);
            rightArm.setPosition(rightArmDownPos);
        }
    }

    public double getLeftSlideEncoder(){
        return leftSlide.getCurrentPosition();
    }

    public double getRightSlideEncoder(){
        return rightSlide.getCurrentPosition();
    }
}
