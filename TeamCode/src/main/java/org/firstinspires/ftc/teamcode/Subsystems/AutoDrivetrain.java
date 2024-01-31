package org.firstinspires.ftc.teamcode.Subsystems;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AutoDrivetrain extends SubsystemBase {
    private final DcMotor leftFront, leftBack, rightFront, rightBack;
    private final BNO055IMU imu;

    public double ticksPerRotation = 537.6; // For AndyMark NeveRest 20
    public double diameter = 7.5; //cm
    public double circumference = Math.PI * diameter;

    public double wheelRatio = 1;

    double oneFootCm = 30.48;

    public AutoDrivetrain(HardwareMap hardwareMap){
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    /*public void moveInchAmount(boolean forward, double inches){
        //using motor encoders
        double driveTrainCorrection = 1;

        double rotationAmount = (oneFootCm / 12) / circumference;
        double totalTicks = rotationAmount * ticksPerRotation * inches * wheelRatio * driveTrainCorrection;
        double oneFoot = rotationAmount * ticksPerRotation * 12 * wheelRatio * driveTrainCorrection;

        double power;

        resetEncoders();

        if(forward){
            while(opModeIsActive() && leftBack.getCurrentPosition() < totalTicks){
                if((totalTicks - leftBack.getCurrentPosition()) < oneFoot) {
                    power = calculateModularPower(.5, .2, (1 / circumference) * (totalTicks - leftBack.getCurrentPosition()), 12, .15);
                }else{
                    power = .5;
                }

                motorsOn(power);
            }
        }else{
            totalTicks = -totalTicks;
            while(opModeIsActive() && leftBack.getCurrentPosition() > totalTicks){
                if((Math.abs(totalTicks) - Math.abs(leftBack.getCurrentPosition())) < oneFoot) {
                    power = calculateModularPower(.5, .2, (1 / circumference) * (Math.abs(totalTicks) - Math.abs(leftBack.getCurrentPosition())), 12, .25);
                }else{
                    power = .5;
                }

                motorsOn(-power);
            }
        }
        motorsOff();
        resetEncoders();
        telemetry.addLine("Pathing finished");
        telemetry.update();
    }*/

    /**
     * Turns all wheels on at specified power
     * @param power Power to run at
     */
    public void motorsOn(double power){
        leftFront.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setPower(power);
    }

    /**
     * Turns all wheels off
     */
    public void motorsOff(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }

    /**
     * Resets all wheel encoders to be at 0 and to run without encoders
     */
    public void resetEncoders() { // Reset all encoder positions
        leftFront.setMode(STOP_AND_RESET_ENCODER);
        leftBack.setMode(STOP_AND_RESET_ENCODER);
        rightFront.setMode(STOP_AND_RESET_ENCODER);
        rightBack.setMode(STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Uses a special Sin wave with a modular steepness (how quickly it drops from its peak) to
     * run smoother and more accurate power changes. By using the Sin wave, the power will be at
     * a lower level for longer than with linear movement allowing encoders to update more frequently
     * per inch travelled and making it more precise.
     * (<a href="https://www.desmos.com/calculator/lyett8tdh1">Desmos Graph Example</a>)
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
     * Returns true for counterclockwise and false for clockwise
     * @param target Target heading to face
     * @return true (CCW) or false (CW)
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
        double absY = Math.abs(y);
        double absZ = Math.abs(z);

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
}
