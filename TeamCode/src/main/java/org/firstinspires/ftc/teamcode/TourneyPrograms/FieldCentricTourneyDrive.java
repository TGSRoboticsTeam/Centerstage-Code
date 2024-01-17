package org.firstinspires.ftc.teamcode.TourneyPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentric Tourney Drive", group = "aa")

public class FieldCentricTourneyDrive extends LinearOpMode {
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
        deposit.setPosition(0.61);

        leftClawRotate.setPosition(0.722);
        rightClawRotate.setPosition(0.148);

        int pixelsReleased = 0;

        boolean liftFlipped = false;
        int maxExtend = -2500;
        double clawPosition = -750;

        double changeInSpeed = 0.2;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, // Change to left if doesn't work
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            // Define joystick controls
            // Drive
            double y   = -gamepad1.left_stick_y;
            double x   =  gamepad1.left_stick_x;
            double rx  =  gamepad1.right_stick_x;

            boolean slowDown = gamepad1.left_bumper;

            // Pixel grabber mechanism
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

            if(gamepad1.dpad_up){
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bots rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (slowDown){
                frontLeftPower  *= changeInSpeed;
                frontRightPower *= changeInSpeed;
                backLeftPower   *= changeInSpeed;
                backRightPower  *= changeInSpeed;
            }

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            // Lift
            lift.setPower(raiseLift-lowerLift);

            if((lift.getPower() <= .05) && (lift.getPower() >= -.05)){
                lift.setPower(0);
            }

            // Grabber
            if (loadPixel){
                deposit.setPosition(0.34);
                pixelsReleased = 0;
            }else if (unloadPixel) {
                if (pixelsReleased == 0) {
                    deposit.setPosition(.52);
                    pixelsReleased = 1;
                }else if (pixelsReleased == 2) {
                    deposit.setPosition(.61);
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

            if (-rightLinearSlide.getCurrentPosition() > 0) {
                rightLinearSlide.setPower(raiseSlides);
            }else if (-rightLinearSlide.getCurrentPosition() < maxExtend) {
                rightLinearSlide.setPower(-lowerSlides);
            }else{
                rightLinearSlide.setPower(raiseSlides - lowerSlides);
            }

            // Deposit rotation
            if (rightLinearSlide.getCurrentPosition() > -clawPosition){
                leftClawRotate.setPosition(0.505);
                rightClawRotate.setPosition(0.358);
            }else{
                leftClawRotate.setPosition(0.73);
                rightClawRotate.setPosition(0.14);
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
        }
    }
}
