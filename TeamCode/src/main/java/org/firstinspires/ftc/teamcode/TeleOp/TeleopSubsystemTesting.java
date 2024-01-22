package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.LinearSlides;

public class TeleopSubsystemTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        LinearSlides linearSlides = new LinearSlides(hardwareMap);

        while(!opModeIsActive()){

        }

        while(opModeIsActive()){
            linearSlides.setPower(gamepad2.right_trigger, gamepad2.left_trigger);
        }
    }
}
