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

        DcMotor activeIntake = hardwareMap.get(DcMotor.class, "active_intake");
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift_mechanism");

        // Claw/Linear slide setup
        Servo hookServo = hardwareMap.get(Servo.class, "claw");
        Servo leftClawRotate = hardwareMap.get(Servo.class, "left_claw_rotation");
        Servo rightClawRotate = hardwareMap.get(Servo.class, "right_claw_rotation");
        DcMotor leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        DcMotor rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        activeIntake.setDirection(DcMotor.Direction.FORWARD);

        leftLinearSlide.setDirection(DcMotor.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotor.Direction.FORWARD);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Makes the motors output their rotation
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            // Define variables
            // The hook position 0 to 1
            double hookPosition = 0.3;
            // The degrees it takes to make the thing automatically go up
            double clawPosition = 200;

            // Define joystick controls
            // Drive
            double leftWheels = -gamepad1.left_stick_y;
            double rightWheels = -gamepad1.right_stick_y;

            // Pixel grabber mechanism
            boolean loadPixel = gamepad2.x;
            boolean unloadPixel = gamepad2.y;
            float linearSlide = gamepad2.right_trigger;
            float linearSlideRetract = gamepad2.left_trigger;

            // Associates buttons/joysticks to motors/servos:
            // Wheels
            leftFrontDrive.setPower(leftWheels);
            leftBackDrive.setPower(leftWheels);
            rightFrontDrive.setPower(rightWheels);
            rightBackDrive.setPower(rightWheels);

            // Linear slide
            leftLinearSlide.setPower(linearSlide - linearSlideRetract);
            rightLinearSlide.setPower(linearSlide - linearSlideRetract);

            // Claw
            if (leftLinearSlide.getCurrentPosition() > clawPosition){
                hookServo.setPosition(hookPosition);
            }else{
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
