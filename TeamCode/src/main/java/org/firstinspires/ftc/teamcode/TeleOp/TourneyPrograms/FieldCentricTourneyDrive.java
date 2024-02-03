package org.firstinspires.ftc.teamcode.TeleOp.TourneyPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Hang;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Plane;

@TeleOp(name = "FieldCentric Tourney Drive", group = "aa")

public class FieldCentricTourneyDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        GamepadEx gamepadEx = new GamepadEx(gamepad2);

        // Motor Setup
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up FtcDashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap);
        Plane plane = new Plane(hardwareMap);
        Hang hang = new Hang(hardwareMap);

        double changeInSpeed = 0.2;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // Change to left if doesn't work
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
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

            hang.setPower(gamepad1.right_trigger, gamepad1.left_trigger);

            if(gamepad1.y){
                hang.moveArm();
            }

            if(gamepad1.x){
                plane.launchPlane();
            }

            linearSlides.setPower(gamepad2.right_trigger, gamepad2.left_trigger);

            if(!gamepad2.b){
                deposit.readyToRetract();
            }

            if(gamepad2.a) {
                deposit.intake();
            }else if(gamepad2.b){
                deposit.outtake();
            }

            if(deposit.isTimerSet()){
                deposit.checkTimer();
            }

            if(gamepad2.right_bumper){
                deposit.closeAligner();
            }else if(gamepad2.left_bumper){
                deposit.openAligner();
            }

            telemetry.addData("Left Slide Encoder: ", linearSlides.getLeftSlideEncoder());
            telemetry.addData("Right Slide Encoder: ", linearSlides.getRightSlideEncoder());
            telemetry.update();
        }
    }
}
