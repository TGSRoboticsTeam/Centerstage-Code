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

package org.firstinspires.ftc.teamcode.AutoClasses;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.AutoClasses.AprilTagDetectionPipe;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@Autonomous(name = "Auto Test Code", group = "Linear Opmode")
public class AutoTesting extends LinearOpMode
{
    // Motor and servo initial setup
    // Motor and servo initial setup
    public DcMotorEx leftDrive;
    public DcMotorEx rightDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightBackDrive;

    public DcMotorEx leftSlide;
    public DcMotorEx rightSlide;

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
    public double pulleyDiameter = 1.10236; // In inches
    public double pulleyCircumference = Math.PI * pulleyDiameter;
    public double tickPerInchForLift = (1 / pulleyCircumference) * ticksPerRotation; // 155.23

    // Servo constants
    static final double zeroPixelPos    = .36;
    static final double onePixelPos     = .26;
    static final double twoPixelPos     = .16;
    public double pixelsHeld = 0;

    static final double armDownPos = .06;
    static final double armUpPos = .17;

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

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode()
    {
        setUpHardware();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Waiting on start");
            telemetry.update();
        }

        /*turnToAngle(45);
        waitTime(3);
        turnToAngle(180);
        waitTime(3);
        turnToAngle(270);
        waitTime(3);
        turnToAngle(90);
        waitTime(3);
        turnToAngle(0);*/

        while(opModeIsActive()) {
            dashboardTelemetry.addData("Heading", getAngle());
            dashboardTelemetry.addData("Optimal change in angle 45", optimalAngleChange(45, getAngle()));
            dashboardTelemetry.addData("Optimal change in angle 90", optimalAngleChange(90, getAngle()));
            dashboardTelemetry.addData("Optimal change in angle 180", optimalAngleChange(180, getAngle()));
            dashboardTelemetry.addData("Optimal change in angle 270", optimalAngleChange(270, getAngle()));
            dashboardTelemetry.update();
        }
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
        double power;

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

                    double headingError = optimalAngleChange(originHeading, getAngle());

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

                    double headingError = optimalAngleChange(originHeading, getAngle());

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
     * @param height Height for slides to move to
     */
    public void moveSlides(double height){
        boolean liftOff = false;
        boolean leftOff = false;
        boolean rightOff = false;

        int targetTick = (int) (height * tickPerInchForLift);
        double fiveInches = (int) (5 * tickPerInchForLift);

        double power = .05;

        slideTarget(targetTick);
        slidePower(power);

        while(!liftOff && opModeIsActive()) {
            /*power = Math.abs(targetTick - leftSlide.getCurrentPosition()) / (fiveInches);

            if(power > 1){
                power = 1;
            }
            if(power < .15){
                power = .15;
            }

            slidePower(power);*/

            if ((leftSlide.getCurrentPosition() > -targetTick - 50 && leftSlide.getCurrentPosition() < -targetTick + 50) || !leftSlide.isBusy() && !leftOff) {
                slidePower(0);
                leftOff = true;
            }

            if ((rightSlide.getCurrentPosition() > targetTick - 50 && rightSlide.getCurrentPosition() < targetTick + 50) || !rightSlide.isBusy() && !rightOff) {
                slidePower(0);
                rightOff = true;
            }

            if(leftSlide.getCurrentPosition() < -1250){
                leftArm.setPosition(armUpPos);
                rightArm.setPosition(armUpPos);
            }else{
                leftArm.setPosition(armDownPos);
                rightArm.setPosition(armDownPos);
            }

            telemetry.addData("Left slide encoder: ", leftSlide.getCurrentPosition());
            telemetry.addData("Right slide encoder: ", rightSlide.getCurrentPosition());
            telemetry.addData("Target Slide Pos: ", targetTick);
            telemetry.update();
        }
    }

    /**
     * Brings deposit down to hold two pixels.
     */
    public void intake(){
        if(pixelsHeld == 0) {
            deposit.setPosition(twoPixelPos);
            pixelsHeld = 2;
        }
    }

    /**
     * Deposits pixels on the backdrop relative to the amount of pixels held.
     */
    public void deposit(){
        if(pixelsHeld == 2){
            deposit.setPosition(onePixelPos);
            pixelsHeld = 1;
        }else if (pixelsHeld == 1){
            deposit.setPosition(zeroPixelPos);
            pixelsHeld = 0;
        }
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
        double originalAngle = getAngle();

        /*
         * Pseudo code for modular speed:
         *
         * if(angleToTarget < 45){
         *     power = angleRemaining / 45;
         * }
         * if(power < .25){
         *      power = .25
         * }
         */

        if(CW) {
            if(originalAngle - 90 < 0 && !(originalAngle < 0)) {
                while (opModeIsActive() && (getAngle() < originalAngle + 5 || getAngle() > originalAngle - 90 + angleCorrectionCW + 360)) {
                    leftVelo(.5);
                    rightVelo(-.5);
                }
            }else{
                while (opModeIsActive() && getAngle() > originalAngle - 90 + angleCorrectionCW && getAngle() < originalAngle + 5) {
                    leftVelo(.5);
                    rightVelo(-.5);
                }
            }
        }else{
            if(originalAngle + 90 > 360) {
                while (opModeIsActive() && (getAngle() > originalAngle - 5 || getAngle() < originalAngle + 90 - angleCorrectionCCW - 360)) {
                    leftVelo(-.5);
                    rightVelo(.5);
                }
            }else{
                while (opModeIsActive() && getAngle() < originalAngle + 90 - angleCorrectionCCW && getAngle() > originalAngle - 5) {
                    leftVelo(-.5);
                    rightVelo(.5);
                }
            }
        }
        motorsOff();
    }

    /**
     * Turns to the specified angle.
     * @param targetAngle Target angle to rotate to
     */
    public void turnToAngle(double targetAngle){
        double originalAngle = getAngle();
        double changeInAngle = optimalAngleChange(targetAngle, originalAngle);

        boolean CW = optimalDirection(targetAngle, originalAngle);

        double power;

        if(CW) {
            if(originalAngle + Math.abs(changeInAngle) + 10 > 360){ // If turning will bring it close to or greater than 360 degrees
                boolean inNewCircle = false;

                while(opModeIsActive() && (!inNewCircle || getAngle() < targetAngle)){
                    double curAngle = getAngle();
                    double remainingDistance = Math.abs(optimalAngleChange(targetAngle, curAngle));

                    if(remainingDistance < 45){
                        power = calculateModularPower(.75, .3, remainingDistance / 3.75, 45 / 3.75, .2);
                    }else{
                        power = 1;
                    }

                    leftDrive.setPower(power);
                    leftBackDrive.setPower(power);
                    rightDrive.setPower(-power);
                    rightBackDrive.setPower(-power);

                    if(curAngle > 0 && curAngle < originalAngle){
                        inNewCircle = true;
                    }
                }
            }else{
                while(opModeIsActive() && getAngle() < targetAngle){
                    double remainingDistance = Math.abs(optimalAngleChange(targetAngle, getAngle()));

                    if(remainingDistance < 45){
                        power = calculateModularPower(.75, .3, remainingDistance / 3.75, 45 / 3.75, .2);
                    }else{
                        power = 1;
                    }

                    leftDrive.setPower(power);
                    leftBackDrive.setPower(power);
                    rightDrive.setPower(-power);
                    rightBackDrive.setPower(-power);
                }
            }

            motorsOff();
        }else{
            if(originalAngle - Math.abs(changeInAngle) - 10 < 0){
                boolean inNewCircle = false;

                while(opModeIsActive() && (!inNewCircle || getAngle() > targetAngle)){
                    double curAngle = getAngle();
                    double remainingDistance = Math.abs(optimalAngleChange(targetAngle, curAngle));

                    if(remainingDistance < 45){
                        power = calculateModularPower(.75, .3, remainingDistance / 3.75, 45 / 3.75, .2);
                    }else{
                        power = 1;
                    }

                    leftDrive.setPower(-power);
                    leftBackDrive.setPower(-power);
                    rightDrive.setPower(power);
                    rightBackDrive.setPower(power);

                    if(curAngle < 360 && curAngle > originalAngle){
                        inNewCircle = true;
                    }
                }
            }else{
                while(opModeIsActive() && getAngle() > targetAngle){
                    double remainingDistance = Math.abs(optimalAngleChange(targetAngle, getAngle()));

                    if(remainingDistance < 45){
                        power = calculateModularPower(.75, .3, remainingDistance / 3.75, 45 / 3.75, .2);
                    }else{
                        power = 1;
                    }

                    leftDrive.setPower(-power);
                    leftBackDrive.setPower(-power);
                    rightDrive.setPower(power);
                    rightBackDrive.setPower(power);
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
        double power;

        double driveTrainCorrection = 1;

        double rotationAmount = (oneFootCm / 12) / circumference;
        double totalTicks = rotationAmount * ticksPerRotation * inches * wheelRatio * driveTrainCorrection;

        // This is used to slow down the robot when within three inches of target position
        double fiveInches = rotationAmount * ticksPerRotation * 5 * wheelRatio * driveTrainCorrection;

        resetEncoders();

        if(forward){
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() < totalTicks){
                power = Math.abs(totalTicks - leftBackDrive.getCurrentPosition()) / (fiveInches);

                if(power > 1){
                    power = 1;
                }

                if(power < .25){
                    power = .25;
                }

                double headingError = optimalAngleChange(originHeading, getAngle());

                leftVelo(power + (headingError * -.1));
                rightVelo(power - (headingError * -.1));

                telemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                telemetry.update();
            }
        }else{
            totalTicks = -totalTicks;
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() > totalTicks){
                power = Math.abs(totalTicks - leftBackDrive.getCurrentPosition()) / (fiveInches);

                if(power > 1){
                    power = 1;
                }

                if(power < .25){
                    power = .25;
                }

                double headingError = optimalAngleChange(originHeading, getAngle());

                leftVelo(-power + (headingError * -.1));
                rightVelo(-power - (headingError * -.1));

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
        double oneFoot = rotationAmount * ticksPerRotation * 12 * wheelRatio * driveTrainCorrection;

        double power;

        resetEncoders();

        if(forward){
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() < totalTicks){
                if((totalTicks - leftBackDrive.getCurrentPosition()) < oneFoot) {
                    power = calculateModularPower(.5, .2, (1 / circumference) * (totalTicks - leftBackDrive.getCurrentPosition()), 12, .15);
                }else{
                    power = .5;
                }

                motorsOn(power);
                dashboardTelemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                dashboardTelemetry.addData("Target Tick:", totalTicks);
                dashboardTelemetry.addData("One Foot Ticks:", oneFoot);
                dashboardTelemetry.addData("Distance Remaining: ", totalTicks - leftBackDrive.getCurrentPosition());
                dashboardTelemetry.addData("Motor power:", leftBackDrive.getPower());
                dashboardTelemetry.addData("Returned power", calculateModularPower(.5, .1, (1 / circumference) * (totalTicks - leftBackDrive.getCurrentPosition()), 12, .15));
                dashboardTelemetry.update();
            }
        }else{
            totalTicks = -totalTicks;
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() > totalTicks){
                if((Math.abs(totalTicks) - Math.abs(leftBackDrive.getCurrentPosition())) < oneFoot) {
                    power = calculateModularPower(.5, .2, (1 / circumference) * (Math.abs(totalTicks) - Math.abs(leftBackDrive.getCurrentPosition())), 12, .25);
                }else{
                    power = .5;
                }

                motorsOn(-power);

                dashboardTelemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                dashboardTelemetry.addData("Target Tick:", totalTicks);
                dashboardTelemetry.addData("One Foot Ticks:", oneFoot);
                dashboardTelemetry.addData("Distance Remaining: ", totalTicks - leftBackDrive.getCurrentPosition());
                dashboardTelemetry.addData("Motor power:", leftBackDrive.getPower());
                dashboardTelemetry.addData("Returned power", calculateModularPower(.5, .1, (1 / circumference) * (Math.abs(totalTicks) - Math.abs(leftBackDrive.getCurrentPosition())), 12, .25));
                dashboardTelemetry.update();
            }
        }
        motorsOff();
        resetEncoders();
        telemetry.addLine("Pathing finished");
        telemetry.update();
    }

    public void leftVelo(double power){ //sets power for left wheels
        leftWheel(power);
        leftBackWheel(power);
    }

    public void rightVelo(double power){ //sets power for right wheels
        rightWheel(power);
        rightBackWheel(power);
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
        while(opModeIsActive() && runtime.seconds() < time){
        }
    }

    /**
     * Turns all wheels off
     */
    public void motorsOff(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
    }

    /**
     * Turns all wheels on at specified power
     * @param power Power to run at
     */
    public void motorsOn(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        rightBackDrive.setPower(power);
        leftBackDrive.setPower(power);
    }

    /**
     * Uses a special Sin wave with a modular steepness (how quickly it drops from its peak) to
     * run smoother and more accurate power changes. By using the Sin wave, the power will be at
     * a lower level for longer than with linear movement allowing encoders to update more frequently
     * per inch travelled and making it more precise.
     * (<a href="https://www.desmos.com/calculator/gsujcy9qjs">Desmos Graph Example</a>)
     * @param maxPower Maximum power allowed to return (0-1)
     * @param minPower Minimum power to return
     * @param remainingDistance Distance to target position
     * @param maxDistance Distance to calculate curve of Sin wave
     * @param kValue Steepness of the Sin wave (0-1; 0 = standard wave, 1 = very steep wave)
     * @return Power to run at in relation to distance from end point
     */
    public double calculateModularPower(double maxPower, double minPower, double remainingDistance, double maxDistance, double kValue){
        if(remainingDistance < 0){
            return minPower;
        }

        double b = 1 / maxDistance;
        double x = -remainingDistance;

        double exponent = Math.pow((2 * (1 - x)), kValue);
        double radians = x * Math.PI - ((maxDistance * Math.PI) / 2);
        double base = maxPower * (.5 + Math.sin(b * radians) / 2);

        double y = Math.pow(base, exponent);

        if(y < minPower){
            y = minPower;
        }

        return y;
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

    /**
     * Sets the target tick position for all drive motors
     * @param target Target tick for wheels to move to
     */
    public void encoderTarget(int target){
        leftDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);
    }

    /**
     * Retrieves IMU heading (+-180 deg), changes the angle to a 360 degree
     * measurement, and returns that heading.
     * @return Current Heading
     */
    public double getAngle(){
        double curHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if(curHeading < 0){
            curHeading = 360 + curHeading;
        }

        return curHeading;
    }

    /**
     * Finds whether turning clockwise or counterclockwise reaches the target heading faster.
     * Returns true for clockwise and false for counterclockwise
     * @param target Target heading to face
     * @return true (CW) or false (CCW)
     */
    public boolean optimalDirection(double target, double curAngle){
        return !(optimalAngleChange(target, curAngle) < 0);
    }

    /**
     * Finds the smallest angle change needed to reach the target angle from current angle.
     * A negative value means a counterclockwise direction, positive is clockwise.
     * @param target Target angle to reach
     * @return Shortest angle to target
     */
    public double optimalAngleChange(double target, double curAngle) {
        double x = target - curAngle;
        double y = target - curAngle - 360;
        double z = target - curAngle + 360;

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

    /**
     * Setup all hardware.
     */
    public void setUpHardware() {
        // Bulk reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        leftSlide = hardwareMap.get(DcMotorEx.class, "left_linear_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_linear_slide");

        deposit = hardwareMap.get(Servo.class, "claw");
        rightArm = hardwareMap.get(Servo.class, "right_claw_rotation");
        leftArm = hardwareMap.get(Servo.class, "left_claw_rotation");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(RUN_USING_ENCODER);
        rightSlide.setMode(RUN_USING_ENCODER);

        leftArm.setPosition(0.722);
        rightArm.setPosition(0.148);
        deposit.setPosition(.61);

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