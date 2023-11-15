/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipe;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Base Auto", group = "Linear Opmode")
public class SimpleAuto extends LinearOpMode
{
    // Motor and servo initial setup
    public DcMotorEx leftDrive;
    public DcMotorEx rightDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightBackDrive;

    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;
    public DcMotor activeIntake;

    public Servo leftArm;
    public Servo rightArm;
    public Servo deposit;

    // Sensors
    BNO055IMU imu;

    public ElapsedTime runtime = new ElapsedTime();

    // Wheel constants
    public double ticksPerRotation = 537.6; // For AndyMark NeveRest 20
    public double diameter = 7.5; //cm
    public double circumference = Math.PI * diameter;

    public double wheelRatio = 1;

    public double angleCorrectionCW = 8.17;
    public double angleCorrectionCCW = 11.26;

    // Linear slide constants
    public double pulleyCircumference = 3.46; // In inches

    public double tickPerInchForLift = ticksPerRotation / pulleyCircumference;

    // Servo constants
    static final double clawOpenPosition = .5;
    static final double clawClosedPosition = 0;

    static final double armDownPos = 0;
    static final double armUpPos = .2;

    // General constants
    double oneFootCm = 30.48;

    OpenCvCamera camera;
    AprilTagDetectionPipe aprilTagDetectionPipe;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST_1 = 7; // Tags from the 36h11 family
    int ID_TAG_OF_INTEREST_2 = 9;
    int ID_TAG_OF_INTEREST_3 = 12;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        setUpHardware();

        while (!isStarted() && !isStopRequested()) {
            if(gamepad2.a){
                activeIntake.setPower(1);
            }else{
                activeIntake.setPower(0);
            }
            if(gamepad2.b){
                deposit.setPosition(0);
            }
        }

        /*moveInchesAtHeading(true, 48);
        waitTime(.5);
        turnNinety(false);
        waitTime(.5);
        moveSlidesAndDrive(true, 55, 10);
        waitTime(.5);
        openDeposit();
        waitTime(.5);
        moveSlides(0);
        waitTime(.5);
        intakeIn(1);
        waitTime(.5);
        closeDeposit();
        waitTime(.5);
        moveSlides(10);
        waitTime(.5);
        openDeposit();
        waitTime(.5);
        moveSlidesAndDrive(false, 4, 0);*/

        moveInchesAtHeading(true, 24);
    }

    /**
     * Moves forward or backward the specified distance while moving linear slides to the specified height
     * @param forward Forward or backward
     * @param distance Distance for robot to move
     * @param height Height to move to
     */
    public void moveSlidesAndDrive(boolean forward, double distance, double height){
        int targetTick = (int) (tickPerInchForLift * height);

        boolean correctionsDone = false;
        boolean motorsOff = false;
        boolean liftOff = false;

        // Drive train calculations
        double driveTrainCorrection = 1;

        double originHeading = getAngle();
        double power = 0;

        double oneRotationDistance = diameter * Math.PI; // In cm
        double rotationAmount = (oneFootCm / 12) / oneRotationDistance;
        double totalTicks = rotationAmount * ticksPerRotation * distance * driveTrainCorrection;
        double threeInches = rotationAmount * ticksPerRotation * 3 * wheelRatio * driveTrainCorrection;

        resetEncoders();

        slideTarget(targetTick);
        slidePower(1);

        runtime.reset();
        if(forward){
            motorsOn(.75);
            while(opModeIsActive() && (!liftOff || !motorsOff)){
                if(!motorsOff) {
                    if (leftBackDrive.getCurrentPosition() >= totalTicks - threeInches) {
                        power = totalTicks / (totalTicks - threeInches);
                        if (power < .25) {
                            power = .25;
                        }
                    } else {
                        power = 1;
                    }

                    double headingError = optimalAngleChange(originHeading);

                    leftVelo(power - (headingError * -.1));
                    rightVelo(power + (headingError * -.1));
                }

                if(leftBackDrive.getCurrentPosition() >= totalTicks){
                    motorsOff();
                    motorsOff = true;
                }

                if(leftSlide.getCurrentPosition() > targetTick - 173 && leftSlide.getCurrentPosition() < targetTick + 173 && !correctionsDone){
                    slideTarget(targetTick);
                    slidePower(.25);
                    correctionsDone = true;
                }else if(leftSlide.getCurrentPosition() > targetTick - 17.3 && leftSlide.getCurrentPosition() < targetTick + 17.3){
                    slidePower(0);
                    liftOff = true;
                }

                if(leftSlide.getCurrentPosition() > 100){
                    leftArm.setPosition(armUpPos);
                    rightArm.setPosition(armUpPos);
                }else{
                    leftArm.setPosition(armDownPos);
                    rightArm.setPosition(armDownPos);
                }

                if(runtime.seconds() > 8){
                    break;
                }
                telemetry.addData("Lift is busy:", leftSlide.isBusy());
                telemetry.update();
            }
        }else{
            totalTicks = -totalTicks;
            motorsOn(-.75);
            while(opModeIsActive() && (!liftOff || !motorsOff)){
                if(!motorsOff) {
                    if (leftBackDrive.getCurrentPosition() >= totalTicks - threeInches) {
                        power = totalTicks / (totalTicks - threeInches);
                        if (power < .25) {
                            power = .25;
                        }
                    } else {
                        power = 1;
                    }

                    double headingError = optimalAngleChange(originHeading);

                    leftVelo(power - (headingError * -.1));
                    rightVelo(power + (headingError * -.1));
                }

                if(leftBackDrive.getCurrentPosition() <= totalTicks){
                    motorsOff();
                    motorsOff = true;
                }

                if(leftSlide.getCurrentPosition() > targetTick - 173 && leftSlide.getCurrentPosition() < targetTick + 173 && !correctionsDone){
                    slideTarget(targetTick);
                    slidePower(.25);
                    correctionsDone = true;
                }else if(leftSlide.getCurrentPosition() > targetTick - 17.3 && leftSlide.getCurrentPosition() < targetTick + 17.3){
                    slidePower(0);
                    liftOff = true;
                }

                if(leftSlide.getCurrentPosition() > 100){
                    leftArm.setPosition(armUpPos);
                    rightArm.setPosition(armUpPos);
                }else{
                    leftArm.setPosition(armDownPos);
                    rightArm.setPosition(armDownPos);
                }

                if(runtime.seconds() > 8){
                    break;
                }
                telemetry.addData("Lift is busy:", leftSlide.isBusy());
                telemetry.update();
            }
        }
        resetEncoders();
    }

    /**
     * Moves the linear slides to the specified height in inches
     * @param height
     */
    public void moveSlides(double height){
        boolean correctionsDone = false;
        boolean liftOff = false;
        int targetTick = (int) (tickPerInchForLift * height);

        slideTarget(targetTick);
        slidePower(1);

        while(!liftOff && opModeIsActive()) {
            if (leftSlide.getCurrentPosition() > targetTick - 173 && leftSlide.getCurrentPosition() < targetTick + 173 && !correctionsDone) {
                slideTarget(targetTick);
                slidePower(.25);
                correctionsDone = true;
            } else if (leftSlide.getCurrentPosition() > targetTick - 17.3 && leftSlide.getCurrentPosition() < targetTick + 17.3) {
                slidePower(0);
                liftOff = true;
            }

            if(leftSlide.getCurrentPosition() > 100){
                leftArm.setPosition(armUpPos);
                rightArm.setPosition(armUpPos);
            }else{
                leftArm.setPosition(armDownPos);
                rightArm.setPosition(armDownPos);
            }
        }
    }

    /**
     * Intakes pixels for Time in seconds
     * @param time Time to intake for
     */
    public void intakeIn(double time){
        resetRuntime();
        while(opModeIsActive() && runtime.seconds() < time){
            activeIntake.setPower(1);
        }
        activeIntake.setPower(0);
    }

    /**
     * Reverse the intake for Time in seconds
     * @param time Time to reverse intake for
     */
    public void intakeOut(double time){
        resetRuntime();
        while(opModeIsActive() && runtime.seconds() < time){
            activeIntake.setPower(-1);
        }
        activeIntake.setPower(0);
    }

    /**
     * Opens end effector claw for deposit on the backboard
     */
    public void openDeposit(){
        deposit.setPosition(clawOpenPosition);
    }

    /**
     * Closes end effector claw for intaking pixels
     */
    public void closeDeposit(){
        deposit.setPosition(clawClosedPosition);
    }

    public void slideTarget(int target){
        leftSlide.setTargetPosition(target);
        rightSlide.setTargetPosition(target);

        leftSlide.setMode(RUN_TO_POSITION);
        rightSlide.setMode(RUN_TO_POSITION);
    }

    public void slidePower(double power){
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void strafeVelo(boolean isLeft, double power, double time){
        //Strafe left or right
        int direction = -1;
        if(isLeft){
            direction = 1;
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time){
            leftWheel(power * direction);
            rightWheel(power * direction * -1);
            leftBackWheel(power * direction * -1);
            rightBackWheel(power * direction);
        }
        if(runtime.seconds() >= time){
            motorsOff();
        }
    }

    /**
     * Turns 90 degrees either clockwise or counter clockwise, depending on value of CW
     * @param CW True or false
     */
    public void turnNinety(boolean CW){
        int adjustment = -1;
        if(CW){
            adjustment *= -1;
        }
        double distancePerRotation = circumference;
        double distanceToTurn = 0.25 * distancePerRotation; // Assuming 90-degree turn
        int ticksToTurn = (int) ((distanceToTurn / circumference) * ticksPerRotation) * adjustment;

        resetEncoders();

        // Set the target position for both motors
        leftBackDrive.setTargetPosition(ticksToTurn);
        rightBackDrive.setTargetPosition(-ticksToTurn); // Negative for opposite direction

        // Set the run mode to RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the desired power for both motors
        double power = 0.5; // Adjust as needed
        motorsOn(power);

        // Wait until both motors reach their target positions
        while (leftBackDrive.isBusy() && rightBackDrive.isBusy()) {
            // Do nothing
        }

        motorsOff();
        resetEncoders();

        /*double originalAngle = getAngle();

        if(CW) {
            if(originalAngle - 90 < 0 && !(originalAngle < 0)) {
                while (opModeIsActive() && (getAngle() < originalAngle + 5 || getAngle() > originalAngle - 90 + angleCorrectionCW + 360)) {
                    leftVelo(.75);
                    rightVelo(-.75);
                }
            }else{
                while (opModeIsActive() && getAngle() > originalAngle - 90 + angleCorrectionCW && getAngle() < originalAngle + 5) {
                    leftVelo(.75);
                    rightVelo(-.75);
                }
            }
        }else{
            if(originalAngle + 90 > 360) {
                while (opModeIsActive() && (getAngle() > originalAngle - 5 || getAngle() < originalAngle + 90 - angleCorrectionCCW - 360)) {
                    leftVelo(-.75);
                    rightVelo(.75);
                }
            }else{
                while (opModeIsActive() && getAngle() < originalAngle + 90 - angleCorrectionCCW && getAngle() > originalAngle - 5) {
                    leftVelo(-.75);
                    rightVelo(.75);
                }
            }
        }*/
        motorsOff();
    }

    /**
     * Turns to the desired angle as specified in targetAngle, assuming starting position is 0 degrees
     * @param targetAngle Target angle to rotate to
     */
    public void turnToAngle(double targetAngle){
        double originalAngle = getAngle();
        double changeInAngle = optimalAngleChange(targetAngle);
        double turningCorrection = (angleCorrectionCW / 90) * changeInAngle;

        boolean CW = optimalDirection(targetAngle);

        if(CW) {
            if(originalAngle - changeInAngle < 0 && !(originalAngle < 0)) {
                while (opModeIsActive() && (getAngle() < originalAngle + 5 || getAngle() > originalAngle - changeInAngle + turningCorrection + 360)) {
                    leftVelo(.75);
                    rightVelo(-.75);
                }
            }else{
                while (opModeIsActive() && getAngle() > originalAngle - changeInAngle  + turningCorrection && getAngle() < originalAngle + 5) {
                    leftVelo(.75);
                    rightVelo(-.75);
                }
            }
        }else{
            if(originalAngle + changeInAngle > 360) {
                while (opModeIsActive() && (getAngle() > originalAngle - 5 || getAngle() < originalAngle + changeInAngle - turningCorrection - 360)) {
                    leftVelo(-.75);
                    rightVelo(.75);
                }
            }else{
                while (opModeIsActive() && getAngle() < originalAngle + changeInAngle - turningCorrection && getAngle() > originalAngle - 5) {
                    leftVelo(-.75);
                    rightVelo(.75);
                }
            }
        }
        motorsOff();
    }

    /**
     * Moves forward or backward for specified amount of inches at heading robot
     * is at on initialization of this function
     * @param forward True or false
     * @param inches Amount to move
     */
    public void moveInchesAtHeading(boolean forward, double inches){
        double originHeading = getAngle();
        double power = 0;
        
        double driveTrainCorrection = 1;

        double rotationAmount = (oneFootCm / 12) / circumference;
        double totalTicks = rotationAmount * ticksPerRotation * inches * wheelRatio * driveTrainCorrection;

        // This is used to slow down the robot when within three inches of target position
        double threeInches = rotationAmount * ticksPerRotation * 3 * wheelRatio * driveTrainCorrection;

        resetEncoders();

        if(forward){
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() < totalTicks){
                if(leftBackDrive.getCurrentPosition() > totalTicks - threeInches){
                    power = totalTicks / (totalTicks - threeInches);
                    if(power < .25){
                        power = .25;
                    }
                }else{
                    power = 1;
                }

                double headingError = optimalAngleChange(originHeading);

                leftVelo(power - (headingError * -.1));
                rightVelo(power + (headingError * -.1));

                telemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                telemetry.update();
            }
        }else{
            totalTicks = -totalTicks;
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() > totalTicks){
                if(leftBackDrive.getCurrentPosition() < totalTicks + threeInches){
                    power = Math.abs(totalTicks / (totalTicks + threeInches));
                    if(power < .25){
                        power = .25;
                    }
                }else{
                    power = 1;
                }

                double headingError = optimalAngleChange(originHeading);

                leftVelo(-power - (headingError * -.1));
                rightVelo(-power + (headingError * -.1));

                telemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                telemetry.update();
            }
        }
        motorsOff();
        resetEncoders();
    }


    /**
     * Moves forward or backward depending on the amount specified in inches
     * @param forward True or false
     * @param inches Amount to move
     */
    public void moveInchAmount(boolean forward, double inches){
        //using motor encoders
        double driveTrainCorrection = 1;

        double rotationAmount = (oneFootCm / 12) / circumference;
        double totalTicks = rotationAmount * ticksPerRotation * inches * wheelRatio * driveTrainCorrection;

        double threeInches = rotationAmount * ticksPerRotation * 3 * wheelRatio * driveTrainCorrection;

        resetEncoders();

        if(forward){
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() < totalTicks){
                if(leftBackDrive.getCurrentPosition() > totalTicks - threeInches){
                    double power = totalTicks / (totalTicks - threeInches);
                    if(power < .25){
                        power = .25;
                    }
                    motorsOn(power);
                }else{
                    motorsOn(1);
                }
                telemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                telemetry.update();
            }
        }else{
            totalTicks = -totalTicks;
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() > totalTicks){
                if(leftBackDrive.getCurrentPosition() < totalTicks + threeInches){
                    double power = Math.abs(totalTicks / (totalTicks + threeInches));
                    if(power < .25){
                        power = .25;
                    }
                    motorsOn(-power);
                }else{
                    motorsOn(-1);
                }
                telemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                telemetry.update();
            }
        }
        motorsOff();
        resetEncoders();
    }

    public void leftVelo(double maxPercent){ //sets power for left wheels
        leftWheel(maxPercent);
        leftBackWheel(maxPercent);
    }

    public void rightVelo(double maxPercent){ //sets power for right wheels
        rightWheel(maxPercent);
        rightBackWheel(maxPercent);
    }

    public void leftWheel(double percent){
        leftDrive.setPower(percent);
    }
    public void rightWheel(double percent){
        rightDrive.setPower(percent);
    }
    public void leftBackWheel(double percent){
        leftBackDrive.setPower(percent);
    }
    public void rightBackWheel(double percent){
        rightBackDrive.setPower(percent);
    }

    //Primitive functions

    /**
     * Pauses all movement for the specified time in seconds
     * @param time How many seconds
     */
    @SuppressWarnings("StatementWithEmptyBody")
    public void waitTime(double time){ // Waits for time (seconds)
        runtime.reset();
        while(opModeIsActive() && runtime.seconds()<time){
        }
    }

    /**
     * Turns all wheels off
     */
    public void motorsOff(){
        leftDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * Turns all wheels on at specified power
     * @param power Percent of power to run at
     */
    public void motorsOn(double power){
        leftDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    /**
     * Resets all wheel encoders to be at 0 and to run without encoders
     */
    public void resetEncoders() { // Reset all encoder positions
        leftDrive.setMode(STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(STOP_AND_RESET_ENCODER);
        rightDrive.setMode(STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderTarget(int target){
        leftDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);
    }

    /**
     * Calculates the angle the robot is at and returns the orientation
     * @return Current Orientation
     */
    public double getAngle(){
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0){
            return 0;
        }
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Finds whether turning clockwise or counterclockwise reaches the target angle faster.
     * Returns true for clockwise and false for counterclockwise
     * @param target Target angle to rotate to
     * @return true (CW) or false (CCW)
     */
    public boolean optimalDirection(double target){
        if(optimalAngleChange(target) < 0){
            return false;
        }else{
            return true;
        }
    }

    /**
     * Finds the smallest angle change needed to reach the target angle from current angle
     * @param target Target angle to reach
     * @return Shortest angle to target
     */
    public double optimalAngleChange(double target) {
        double x = target - getAngle();
        double y = target - getAngle() - 360;
        double z = target - getAngle() + 360;

        double absX = Math.abs(x);
        double absZ = Math.abs(y);
        double absY = Math.abs(z);

        double min = absX;

        if(absZ < min){
            min = absZ;
        }
        if(absY < min){
            min = absY;
        }

        if(min == absX){
            return x;
        }else if(min == absY){
            return y;
        }else{
            return z;
        }
    }

    public void setUpHardware() { // Assigns motor names in phone to the objects in code
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        leftSlide = hardwareMap.get(DcMotorEx.class, "left_linear_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_linear_slide");
        activeIntake = hardwareMap.get(DcMotor.class, "active_intake");

        deposit = hardwareMap.get(Servo.class, "claw");
        leftArm = hardwareMap.get(Servo.class, "right_claw_rotation");
        rightArm = hardwareMap.get(Servo.class, "left_claw_rotation");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setDirection(Servo.Direction.FORWARD);
        rightArm.setDirection(Servo.Direction.REVERSE);

        activeIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(RUN_USING_ENCODER);
        rightSlide.setMode(RUN_USING_ENCODER);

        leftArm.setPosition(0);
        rightArm.setPosition(0);
        deposit.setPosition(.5);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        if(detection.id == 7){
            telemetry.addLine("\nDetected tag location = 1");
        }else if(detection.id == 9){
            telemetry.addLine("\nDetected tag location = 2");
        }else if(detection.id == 12){
            telemetry.addLine("\nDetected tag location = 3");
        }
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.x)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.y)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.z)));
    }
}