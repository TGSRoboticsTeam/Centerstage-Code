package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Yael Drive", group = "Yael Drive")

@Config
public class YaelDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Motor Setup
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        DcMotor lift = hardwareMap.get(DcMotor.class, "lift_mechanism");

        // Grabber/Linear slide setup
        ServoImpl liftServo = hardwareMap.get(ServoImpl.class, "lift_flipper");

        // Claw/Linear slide setup
        ServoImpl deposit = hardwareMap.get(ServoImpl.class, "claw");
        Servo leftClawRotate = hardwareMap.get(Servo.class, "left_claw_rotation");
        Servo rightClawRotate = hardwareMap.get(Servo.class, "right_claw_rotation");

        DcMotor leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        DcMotor rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        Servo plane = hardwareMap.get(Servo.class, "plane_launcher");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.REVERSE);

        leftLinearSlide.setDirection(DcMotor.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotor.Direction.REVERSE);

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

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up FtcDashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Servo starting positions
        liftServo.setPosition(.44);
        deposit.setPosition(0.25);

        leftClawRotate.setPosition(0.83);
        rightClawRotate.setPosition(0.7);

        plane.setPosition(0);

        int pixelsReleased = 0;

        boolean liftFlipped = false;
        int maxExtend = -2500;
        double clawPosition = -750;

        double changeInSpeed = 0.2;

        // Need this so that the code will stay initialized until you hit play on the phone
        while (!isStarted()) {
            liftServo.setPosition(.44);
            deposit.setPosition(0.61);

            leftClawRotate.setPosition(0.722);
            rightClawRotate.setPosition(0.148);
        }

        while (opModeIsActive()) {
            /* Define control variables */

            // Drive
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            boolean slowDown = gamepad1.left_bumper;

            boolean flipLift = gamepad1.y;
            boolean unflipLift = gamepad1.b;
            float raiseLift = gamepad1.right_trigger;
            float lowerLift = gamepad1.left_trigger;

            float raiseSlides = gamepad2.right_trigger;
            float lowerSlides = gamepad2.left_trigger;

            // Lift
            boolean loadPixel = gamepad2.a;
            boolean unloadPixel = gamepad2.b;

            boolean launchPlane = gamepad2.x;

            if (axial <= 0.1 && axial >= -0.1) {
                axial = 0;
            }

            if (lateral <= 0.1 && lateral >= -0.1) {
                lateral = 0;
            }

            if (yaw <= 0.1 && yaw >= -0.1) {
                yaw = 0;
            }

            // Gives the joystick commands purpose, "mecanum" wheel stuff or whatever
            double rightFront = axial - lateral - yaw;
            double rightBack  = axial + lateral - yaw;
            double leftBack   = axial - lateral + yaw;
            double leftFront  = axial + lateral + yaw;
            double max;

            max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            max = Math.max(max, Math.abs(leftBack));
            max = Math.max(max, Math.abs(rightBack));

            if (max > 1.0) {
                leftFront  /= max;
                rightFront /= max;
                leftBack   /= max;
                rightBack  /= max;
            }

            if (slowDown){
                leftFront  *= changeInSpeed;
                rightFront *= changeInSpeed;
                leftBack   *= changeInSpeed;
                rightBack  *= changeInSpeed;
            }

            // Associates buttons/joysticks to motors/servos:
            // Wheels
            leftFrontDrive.setPower(leftFront);
            leftBackDrive.setPower(leftBack);
            rightFrontDrive.setPower(rightFront);
            rightBackDrive.setPower(rightBack);

            // Lift
            lift.setPower(raiseLift-lowerLift);

            if((lift.getPower() <= .05) && (lift.getPower() >= -.05)){
                lift.setPower(0);
            }

            // Grabber
            // Makes it so you can release pixels individually
            if (loadPixel){
                deposit.setPosition(0.04);
                pixelsReleased = 0;
            }else if (unloadPixel) {
                if (pixelsReleased == 0) {
                    deposit.setPosition(.15);
                    pixelsReleased = 1;
                }else if (pixelsReleased == 2) {
                    deposit.setPosition(.25);
                }
            }

            if (!loadPixel && !unloadPixel && pixelsReleased == 1) {
                pixelsReleased = 2;
            }

            // Launches Plane
            if (launchPlane) {
                plane.setPosition(1);
            }

            if (leftLinearSlide.getCurrentPosition() > 0) {
                leftLinearSlide.setPower(raiseSlides);
            }else if (leftLinearSlide.getCurrentPosition() < maxExtend) {
                leftLinearSlide.setPower(-lowerSlides);
            }else{
                leftLinearSlide.setPower(raiseSlides - lowerSlides);
            }

            if (-rightLinearSlide.getCurrentPosition() > 0) {
                rightLinearSlide.setPower(raiseSlides);
            }else if (-rightLinearSlide.getCurrentPosition() < maxExtend) {
                rightLinearSlide.setPower(-lowerSlides);
            }else{
                rightLinearSlide.setPower(raiseSlides - lowerSlides);
            }

            // Deposit rotation

            if (rightLinearSlide.getCurrentPosition() > -clawPosition){
                leftClawRotate.setPosition(0.61);
                rightClawRotate.setPosition(0.92);
            }else{
                leftClawRotate.setPosition(0.83);
                rightClawRotate.setPosition(0.7);
            }

            /*if(gamepad1.dpad_up){
                leftClawRotate.setPosition(0.505);
                rightClawRotate.setPosition(0.358);
            }else if(gamepad1.dpad_down){
                leftClawRotate.setPosition(0.722);
                rightClawRotate.setPosition(0.148);
            }*/

            if(flipLift && !liftFlipped){
                liftServo.setPosition(.25);
                liftFlipped = true;
            }else if (unflipLift) {
                liftServo.setPosition(.44);
                liftFlipped = false;
            }

            lift.setPower(raiseLift-lowerLift);

            if((lift.getPower() <= .05) && (lift.getPower() >= -.05)){
                lift.setPower(0);
            }

            //dashboardTelemetry.addData("Left Slide Pos: ", leftLinearSlide.getCurrentPosition());
            //dashboardTelemetry.addData("Right Slide Pos: ", rightLinearSlide.getCurrentPosition());
            dashboardTelemetry.addData("Right Servo", rightClawRotate.getPosition());
            dashboardTelemetry.addData("Left Servo", leftClawRotate.getPosition());
            dashboardTelemetry.addData("Deposit", deposit.getPosition());
            dashboardTelemetry.update();
        }
    }
}
