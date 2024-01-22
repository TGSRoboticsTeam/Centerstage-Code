package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.LinearSlides;
import org.firstinspires.ftc.teamcode.TeleOp.Subsystems.Plane;

@TeleOp(name = "Subsystem Testing", group = "Testing")

public class TeleopSubsystemTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap);
        Plane plane = new Plane(hardwareMap);

        while(!opModeIsActive()){

        }

        while(opModeIsActive()){
            linearSlides.setPower(gamepad2.right_trigger, gamepad2.left_trigger);

            if(gamepad2.a) {
                deposit.intake();
            }else if(gamepad2.b){
                deposit.outtake();
            }

            if(gamepad2.x){
                plane.launchPlane();
            }
        }
    }
}
