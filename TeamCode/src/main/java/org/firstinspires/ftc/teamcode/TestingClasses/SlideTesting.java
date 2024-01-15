package org.firstinspires.ftc.teamcode.TestingClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Slide Testing", group = "Testing")

public class SlideTesting extends LinearOpMode {

    @Override
    public void runOpMode(){
        DcMotor leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        DcMotor rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        leftLinearSlide.setDirection(DcMotor.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotor.Direction.REVERSE);

        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        while(!opModeIsActive()){

        }

        while(opModeIsActive()){
            leftLinearSlide.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            rightLinearSlide.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }
}
