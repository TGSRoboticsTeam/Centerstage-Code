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
<<<<<<< HEAD
        Servo launcher = hardwareMap.get(ServoImpl.class, "plane_launcher");

        // Grabber/Linear slide setup
        ServoImpl grabberServo = hardwareMap.get(ServoImpl.class, "grabber");
        DcMotor leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
=======
        Servo liftRotation = hardwareMap.get(Servo.class, "lift_rotation");

        // Claw/Linear slide setup
        ServoImpl hookServo = hardwareMap.get(ServoImpl.class, "claw");
        Servo leftClawRotate = hardwareMap.get(Servo.class, "left_claw_rotation");
        Servo rightClawRotate = hardwareMap.get(Servo.class, "right_claw_rotation");

        DcMotor leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        DcMotor rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        Servo plane = hardwareMap.get(Servo.class, "plane_launcher");
>>>>>>> master

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        lift.setDirection(DcMotor.Direction.FORWARD);

<<<<<<< HEAD
=======
        leftClawRotate.setDirection(Servo.Direction.REVERSE);
        rightClawRotate.setDirection(Servo.Direction.FORWARD);

>>>>>>> master
        leftLinearSlide.setDirection(DcMotor.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotor.Direction.FORWARD);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
<<<<<<< HEAD

        // Makes the motors output their rotation
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // servo starting position
        grabberServo.setPosition(0);

        int pixelsReleased = 0;
=======
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Makes the motors output their rotation
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // servo starting position
        hookServo.setPosition(0);
        plane.setPosition(0);

        boolean liftFlipped = false;
>>>>>>> master

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            // Define variables
            // The hook position 0 to 1
            double hookPosition = 0.1;
            // The degrees it takes to make the thing automatically go up
            double clawPosition = -1250;

            // Define joystick controls
            // Drive
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

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

            // Pixel grabber mechanism
<<<<<<< HEAD
            boolean grabPixel = gamepad2.a;
            boolean releasePixel = gamepad2.b;
            float linearSlide = -gamepad2.left_stick_y;

            // Lift
            float liftExtend = gamepad2.right_trigger - gamepad2.left_trigger;
            double liftMaxExtend = 10000;
=======
            boolean loadPixel = gamepad2.a;
            boolean unloadPixel = gamepad2.b;

            boolean launchPlane = gamepad2.x;

            boolean flipLift = gamepad1.a;
            float raiseLift = gamepad1.right_trigger;
            float lowerLift = gamepad1.left_trigger;

            double moveSlide = -gamepad2.left_stick_y;
            double slidesMaxExtend = 10000;
>>>>>>> master

            // Plane launcher
            boolean launch = gamepad2.x;

            // Associates buttons/joysticks to motors/servos:
            // Wheels
            leftFrontDrive.setPower(leftFront);
            leftBackDrive.setPower(leftBack);
            rightFrontDrive.setPower(rightFront);
            rightBackDrive.setPower(rightBack);

            // Linear slide
<<<<<<< HEAD
            int maxExtend = 3000;

            if (-leftLinearSlide.getCurrentPosition() <= 0) {
                if (linearSlide > 0) {
                    leftLinearSlide.setPower(linearSlide);
                }
            }else if (-leftLinearSlide.getCurrentPosition() >= maxExtend) {
                if (linearSlide < 0) {
                    leftLinearSlide.setPower(linearSlide);
                }
            }else{
                leftLinearSlide.setPower(linearSlide);
            }

            // Lift
            if (leftLinearSlide.getCurrentPosition() > 0) {
                lift.setPower(linearSlide);
            }

            // Grabber
            if (grabPixel){
                grabberServo.setPosition(1);
                pixelsReleased = 0;
            }else if (releasePixel) {
                if (pixelsReleased == 0) {
                    grabberServo.setPosition(0.5);
                    pixelsReleased = 1;
                }else if (pixelsReleased == 2) {
                    grabberServo.setPosition(0);
                }
            }

            if (!grabPixel && !releasePixel && pixelsReleased == 1) {
                pixelsReleased = 2;
            }

            // Launches Plane
            if (launch) {
                launcher.setPosition(0.1);
=======
            int maxExtend = -3000;

            leftLinearSlide.setPower(moveSlide);
            rightLinearSlide.setPower(moveSlide);

            if (leftLinearSlide.getCurrentPosition() > 0) {
                leftLinearSlide.setPower(Math.abs(moveSlide));
            }else if (leftLinearSlide.getCurrentPosition() < maxExtend) {
                leftLinearSlide.setPower(-Math.abs(moveSlide));
            }else{
                leftLinearSlide.setPower(moveSlide);
            }

            if (rightLinearSlide.getCurrentPosition() > 0) {
                rightLinearSlide.setPower(Math.abs(moveSlide));
            }else if (leftLinearSlide.getCurrentPosition() < maxExtend) {
                rightLinearSlide.setPower(-Math.abs(moveSlide));
            }else{
                rightLinearSlide.setPower(moveSlide);
            }

            // Deposit rotation
            if (leftLinearSlide.getCurrentPosition() < clawPosition){
                leftClawRotate.setPosition(0.5);
                rightClawRotate.setPosition(0.5);
>>>>>>> master
            }else{
                launcher.setPosition(0);
            }

<<<<<<< HEAD
            // Hook
            // Isaac can you add some code here? Ihdk how this hook thing works...

=======
            // Claw-Hook
            if (loadPixel){
                hookServo.setPosition(0.47);
            }else if (unloadPixel) {
                hookServo.setPosition(0);
            }

            if(flipLift && !liftFlipped){
                liftRotation.setPosition(.5);
                liftFlipped = true;
            }

            if(liftFlipped){
                lift.setPower(raiseLift - lowerLift);
            }
>>>>>>> master

            telemetry.addData("Lift encoder", leftLinearSlide.getCurrentPosition());
            telemetry.addData("Servo voltage", grabberServo.getConnectionInfo());
            telemetry.update();
        }

    }
}
