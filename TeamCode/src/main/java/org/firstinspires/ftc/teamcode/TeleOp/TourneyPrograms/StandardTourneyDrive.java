package org.firstinspires.ftc.teamcode.TeleOp.TourneyPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Hang;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.LinearSlides;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Plane;

@TeleOp(name = "Standard Tourney Drive", group = "aa")

public class StandardTourneyDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
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

        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap);
        Plane plane = new Plane(hardwareMap);
        Hang hang = new Hang(hardwareMap);

        // Set up FtcDashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        double changeInSpeed = 0.2;

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            // Define joystick controls
            // Drive
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            boolean slowDown = gamepad1.left_bumper;

            if (axial <= 0.1 && axial >= -0.1) {
                axial = 0;
            }

            if (lateral <= 0.1 && lateral >= -0.1) {
                lateral = 0;
            }

            if (yaw <= 0.1 && yaw >= -0.1) {
                yaw = 0;
            }

            // Gives the joystick commands purpose
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
        }
    }
}
