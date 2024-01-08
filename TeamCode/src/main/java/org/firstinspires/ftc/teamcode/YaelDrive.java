package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImpl;
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

        lift.setDirection(DcMotor.Direction.FORWARD);

        leftClawRotate.setDirection(Servo.Direction.REVERSE);
        rightClawRotate.setDirection(Servo.Direction.FORWARD);

        leftLinearSlide.setDirection(DcMotor.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotor.Direction.FORWARD);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int pixelsReleased = 0;

        boolean liftFlipped = false;

        // Makes the motors output their rotation
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo starting positions
        liftServo.setPosition(.44);
        deposit.setPosition(.35);

        leftClawRotate.setPosition(.03);

        // Need this so that the code will stay initialized until you hit play on the phone
        while (!isStarted()) {
            telemetry.addData("Servo Position: ", deposit.getPosition());
            telemetry.update();
        }

        while (opModeIsActive()) {
            /* Define control variables */
            // Grabber vars

            // The two following variables are negative because the left linear slides encoder
            // goes negative as it goes up, and that's what we had been using.
            int maxExtend = -3000;

            // This height worked when testing servo positions.
            double clawPosition = -1250;

            // I'll explain limiting max power when we get back
            double motorMax = 0.85;

            // Drive
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            boolean slowDown = gamepad1.left_bumper;

            float raiseSlides = gamepad2.right_trigger;
            float lowerSlides = gamepad2.left_trigger;

            // Lift
            boolean loadPixel = gamepad2.a;
            boolean unloadPixel = gamepad2.b;

            boolean launchPlane = gamepad2.x;

            boolean flipLift = gamepad1.y;
            boolean unflipLift = gamepad1.b;
            float raiseLift = gamepad1.right_trigger;
            float lowerLift = gamepad1.left_trigger;

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
            // Multipiyes the wheels power by a decimal value to lower the speed;
            double changeInSpeed = 0.2;


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
                deposit.setPosition(0.16);
                pixelsReleased = 0;
            }else if (unloadPixel) {
                if (pixelsReleased == 0) {
                    deposit.setPosition(.26);
                    pixelsReleased = 1;
                }else if (pixelsReleased == 2) {
                    deposit.setPosition(.36);
                }
            }

            if (!loadPixel && !unloadPixel && pixelsReleased == 1) {
                pixelsReleased = 2;
            }

            // Launches Plane
            if (launchPlane) {
                plane.setPosition(0);
            }

            if (leftLinearSlide.getCurrentPosition() > 0) {
                leftLinearSlide.setPower(raiseSlides);
            }else if (leftLinearSlide.getCurrentPosition() < maxExtend) {
                leftLinearSlide.setPower(-lowerSlides);
            }else{
                leftLinearSlide.setPower(raiseSlides - lowerSlides);
            }

            /*if (rightLinearSlide.getCurrentPosition() < 0) {
                rightLinearSlide.setPower(raiseSlides);
            }else if (leftLinearSlide.getCurrentPosition() > -maxExtend) {
                rightLinearSlide.setPower(-lowerSlides);
            }else{
                rightLinearSlide.setPower(raiseSlides - lowerSlides);
            }*/

            // Deposit rotation
            if (leftLinearSlide.getCurrentPosition() < clawPosition){
                leftClawRotate.setPosition(0.26);
                //rightClawRotate.setPosition(0.5);
            }else{
                leftClawRotate.setPosition(.03);
                //rightClawRotate.setPosition(0);
            }

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

            telemetry.addData("Left Slide Pos: ", leftLinearSlide.getCurrentPosition());
            telemetry.addData("Right Slide Pos: ", rightLinearSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}
