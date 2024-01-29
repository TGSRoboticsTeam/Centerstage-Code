package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Deposit;
import org.firstinspires.ftc.teamcode.Subsystems.Hang;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlides;
import org.firstinspires.ftc.teamcode.Subsystems.Plane;

@TeleOp(name = "Subsystem Testing", group = "Testing")
@Disabled
public class TeleOpSubsystemTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        LinearSlides linearSlides = new LinearSlides(hardwareMap);
        Deposit deposit = new Deposit(hardwareMap);
        Plane plane = new Plane(hardwareMap);
        Hang hang = new Hang(hardwareMap);

        while(!opModeIsActive()){
            telemetry.addLine("Waiting on start.");
            telemetry.update();
        }

        while(opModeIsActive()){
            hang.setPower(gamepad1.right_trigger, gamepad1.left_trigger);

            if(gamepad1.y){
                hang.moveArm();
            }

            if(gamepad1.x){
                plane.launchPlane();
            }

            linearSlides.setPower(gamepad2.right_trigger, gamepad2.left_trigger);

            if(gamepad2.a) {
                deposit.intake();
            }else if(gamepad2.b){
                deposit.outtake();
            }
        }
    }
}
