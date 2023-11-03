package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Yael Drive", group = "Yael Drive")

public class YaelDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Motor Setup
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Claw/Linear slide setup
        Servo hookServo = hardwareMap.get(Servo.class, "claw");
        Servo leftClawRotate = hardwareMap.get(Servo.class, "left_claw_rotation");
        Servo rightClawRotate = hardwareMap.get(Servo.class, "right_claw_rotation");
        DcMotor leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        DcMotor rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            // Define variables
            // The hook position 0 to 1
            double hookPosition = 0.3;
            // The minimum the joysticks have to be pressed to register as an input
            double min = 0.5;

            // Define joystick controls
            // Drive
            double leftWheels = -gamepad1.left_stick_y;
            double rightWheels = -gamepad1.right_stick_y;

            // Pixel grabber mechanism
            boolean hook = gamepad2.a;
            boolean unhook = gamepad2.b;
            boolean loadPixel = gamepad2.x;
            boolean unloadPixel = gamepad2.y;
            double linearSlide = gamepad2.right_trigger;
            double linearSlideRetract = -gamepad2.left_trigger;

            // Associates buttons/joysticks to motors/servos:
            // Wheels
            if (Math.abs(leftWheels) > min && Math.abs(rightWheels) > min) {
                leftFrontDrive.setPower(leftWheels);
                leftBackDrive.setPower(leftWheels);
                rightFrontDrive.setPower(rightWheels);
                rightBackDrive.setPower(rightWheels);
            }

            // Linear slide
            leftLinearSlide.setPower(linearSlide - linearSlideRetract);
            rightLinearSlide.setPower(linearSlide - linearSlideRetract);

            // Claw
            if (hook){
                hookServo.setPosition(hookPosition);
            }else if (unhook){
                hookServo.setPosition(0);
            }

            // Claw direction
            if (loadPixel){
                leftClawRotate.setPosition(0.5);
                rightClawRotate.setPosition(0.5);
            }else if(unloadPixel){
                leftClawRotate.setPosition(0);
                rightClawRotate.setPosition(0);
            }

        }

    }
}
