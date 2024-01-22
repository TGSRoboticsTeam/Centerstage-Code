/*
 * I will be working on this program to have some code to run if needed at the tournament,
 * however I would much prefer to have code written by one of you which has been interested
 * in coding. Please reference this code if you are struggling to code a part of it, but don't
 * copy it word for word (well, if you understand it feel free to, but otherwise don't). If
 * you're confused at how it works, you can also just ask me at some point. I will try my
 * best to add comments to the code to make it more understandable.
 */


package org.firstinspires.ftc.teamcode.RandomClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled

@TeleOp(name="TeleOp Code", group="Linear Opmode")

public class IsaacDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        // These are the motors that will power the linear slides and active intake. We will
        // need to add the names of them into the driver station phone.
        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "left_linear_slide");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class, "right_linear_slide");
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift_mechanism");
        DcMotor activeIntake = hardwareMap.get(DcMotor.class, "active_intake");


        // These are the servos needed for our deposit and arm attached to the linear slide.
        Servo deposit = hardwareMap.get(Servo.class, "claw");
        Servo leftArm = hardwareMap.get(Servo.class, "right_claw_rotation");
        Servo rightArm = hardwareMap.get(Servo.class, "left_claw_rotation");


        // These commands tell the motor to brake when there is no set power it should run at.
        // It just makes it so it would require more outside force to move the linear slides
        // when the robot is powered on.
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Resetting the encoders for the linear slide motors to set the bottom of the
        // slides as 0 ticks. This will be used later to automatically flip the deposit
        // when the slides have passed a certain height.
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // This sets the proper direction of rotation for the drivetrain motors, as two of them
        // would actually be reversed since they are facing the other direction. Test this using
        // the SimpleDrive program first, and use the button option to test the wheels individually
        // and to make sure they are wired correctly.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        // Same situation for linear slides, active intake is there just in case we need to flip it
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.REVERSE);

        activeIntake.setDirection(DcMotorSimple.Direction.FORWARD);


        // Setting servos to be at proper location on init
        leftArm.setPosition(0); // This will be tuned to be facing exactly down
        rightArm.setPosition(0);
        deposit.setPosition(.5); // Init position for this should be at open

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Up to this point, all we've been doing is setting up the motors and servos. The while
        // loop directly below this comment now forces the robot to not do anything until until you
        // hit play on the phone (only runs in the period between hitting "init" and "play).
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("FL Encoder:", leftFrontDrive.getCurrentPosition());
            telemetry.addData("BL Encoder:", leftBackDrive.getCurrentPosition());
            telemetry.addData("FR Encoder:", rightFrontDrive.getCurrentPosition());
            telemetry.addData("BR Encoder:", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
        runtime.reset();

        // This while loop will run once you've hit play on the phone, and stop once you've hit stop.
        while (opModeIsActive()) {
            double max;

            // This is all the controller inputs. Gamepad1 usually is for the driver, and gamepad2 is
            // for the person controlling the different mechanisms.
            double axial        = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral      =  gamepad1.left_stick_x;
            double yaw          =  gamepad1.right_stick_x;
            boolean intakeIn    =  gamepad1.right_bumper;
            boolean intakeOut   =  gamepad1.left_bumper; // On the off chance we intake more than two pixels, use this to send the extras back out

            double slideControl = -gamepad2.left_stick_y; // By using the joystick, we can move the slides up and down at different speeds to be more precise
            boolean grabPixel   =  gamepad2.right_bumper;
            boolean dropPixel   =  gamepad2.left_bumper;
            // Note: Ideally we will make the arm turn around completely automatically,
            // so we don't need to assign any controls for it.


            // This code checks to see if you are trying to run the active intake, and
            // then turns the motor on if you are. It prioritizes intaking pixels over
            // sending them out in case you are hitting both at the same time.
            if(intakeIn){
                activeIntake.setPower(1); // Runs the motor forward
            }else if(intakeOut){
                activeIntake.setPower(-1); // Runs the motor backward
            }else{
                activeIntake.setPower(0); // Stops the motor
            }


            // This code checks to see if you are grab or drop a pixel, and moves the servo accordingly
            if(grabPixel){
                deposit.setPosition(0); // Grabs the pixel
            }else if(dropPixel){
                deposit.setPosition(.5); // Drops the pixel
            }


            // This code runs the linear slides. If the value for slide control is
            // positive, then it moves up, if it is negative, it moves down. It also
            // makes sure the motors minimum speed is 25% of its max speed so that the
            // motor will have enough power to lift the linear slide. Depending on how
            // much power is actually needed, that number may change to be lower or higher.
            if(slideControl >= .1 || slideControl <= -.1){ // Checks to see if the joystick has moved, set at .1 to account for potential joystick drift
                double power = Math.abs(slideControl);
                if(power < .25){
                    power = .25;
                }
                if(slideControl >= .1){ // Long term, we will want to set a max height for the linear slides so we cant extend to far and end up going down again
                    leftSlide.setPower(power);
                    rightSlide.setPower(power);
                }else if(leftSlide.getCurrentPosition() > 0){ // Won't go down if it is already at or below the minimum height
                    leftSlide.setPower(-power);
                    rightSlide.setPower(-power);
                }
            }else{ // Make sure to set power at 0 if you aren't doing anything
                rightSlide.setPower(0);
                leftSlide.setPower(0);
            }


            // This just checks to make sure the linear slides are not to far down,
            // and if they are it slowly brings them back up until the encoders are
            // greater than or equal to 0.
            /*
            if(leftSlide.getCurrentPosition() < 0 && leftSlide.getPower() == 0){
                leftSlide.setPower(.1); // This might be to slow, but we don't want it to be to fast
            }
            if(rightSlide.getCurrentPosition() < 0 && rightSlide.getPower() == 0){
                rightSlide.setPower(.1);
            }
            */


            // Checks to see if the linear slides have lifted high enough, and if so
            // it flips the arm around so the deposit is facing the board correctly.
            if(leftSlide.getCurrentPosition() > 100){ // Completely arbitrary, will need to test to find correct height
                leftArm.setPosition(.2); // Also completely arbitrary
                rightArm.setPosition(.2);
            }else if(leftSlide.getCurrentPosition() < 100){
                leftArm.setPosition(0);
                rightArm.setPosition(0);
            }


            // The rest of the code here is for running the drivetrain motors

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}