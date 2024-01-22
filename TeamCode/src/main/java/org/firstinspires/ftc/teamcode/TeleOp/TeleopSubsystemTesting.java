package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.LinearSlides;

public class TeleopSubsystemTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap);

        while(!opModeIsActive()){

        }

        while(opModeIsActive()){
            linearSlides.setPower(gamepad2.right_trigger, gamepad2.left_trigger);

            if(gamepad2.a) {
                deposit.intake();
            }else if(gamepad2.b){
                deposit.outtake();
            }
        }
    }
}
