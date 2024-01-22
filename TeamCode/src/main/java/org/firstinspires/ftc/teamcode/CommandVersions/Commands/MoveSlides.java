package org.firstinspires.ftc.teamcode.CommandVersions.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.CommandVersions.Subsystems.DualLinearSlide;
import java.util.function.DoubleSupplier;

public class MoveSlides extends CommandBase{

    private final DualLinearSlide slides;
    private final DoubleSupplier power;

    public MoveSlides(DualLinearSlide slides, DoubleSupplier power){
        this.slides = slides;
        this.power = power;

        addRequirements(slides);
    }

    @Override
    public void execute(){
        slides.setPower(power.getAsDouble());
    }
}