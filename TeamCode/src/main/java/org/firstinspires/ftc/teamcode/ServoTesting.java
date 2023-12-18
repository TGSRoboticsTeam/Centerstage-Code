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

@Disabled
public class ServoTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        Servo servoOne = hardwareMap.get(Servo.class, "servo_1");
        Servo servoTwo = hardwareMap.get(Servo.class, "servo_2");
        Servo servoThree = hardwareMap.get(Servo.class, "servo_3");

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            boolean servo1 = gamepad1.a;
            boolean servo2 = gamepad1.b;
            boolean servo3 = gamepad1.x;

            if (servo1) {
                servoOne.setPosition(0.5);
            }else{
                servoOne.setPosition(0);
            }

            if (servo2) {
                servoTwo.setPosition(0.5);
            }else{
                servoTwo.setPosition(0);
            }

            if (servo3) {
                servoThree.setPosition(0.5);
            }else{
                servoThree.setPosition(0);
            }
        }
    }
}
