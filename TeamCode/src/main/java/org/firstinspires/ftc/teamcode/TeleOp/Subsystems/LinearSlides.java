package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LinearSlides extends SubsystemBase {

    private final DcMotor leftSlide, rightSlide;
    private final Servo leftArm, rightArm;

    int maxHeight = -2500;

    double armRotationPos = -750;

    double leftArmUpPos = .61;
    double leftArmDownPos = .83;

    double rightArmUpPos = .92;
    double rightArmDownPos = .7;

    public LinearSlides(HardwareMap hardwareMap){
        leftSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        leftArm = hardwareMap.get(Servo.class, "left_claw_rotation");
        rightArm = hardwareMap.get(Servo.class, "right_claw_rotation");
    }

    public void setPower(double upPower, double downPower){
        if (leftSlide.getCurrentPosition() > 0) {
            leftSlide.setPower(upPower);
        }else if (leftSlide.getCurrentPosition() < maxHeight) {
            leftSlide.setPower(-downPower);
        }else{
            leftSlide.setPower(upPower - downPower);
        }

        if (-rightSlide.getCurrentPosition() > 0) {
            rightSlide.setPower(upPower);
        }else if (-rightSlide.getCurrentPosition() < maxHeight) {
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
}
