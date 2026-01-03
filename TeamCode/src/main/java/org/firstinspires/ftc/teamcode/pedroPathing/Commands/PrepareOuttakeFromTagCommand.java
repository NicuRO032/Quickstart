package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;

public class PrepareOuttakeFromTagCommand extends CommandBase {
    private final CarouselSubsystem1 carousel;
    private final int tagId;

    public PrepareOuttakeFromTagCommand(CarouselSubsystem1 carousel, int tagId) {
        this.carousel = carousel;
        this.tagId = tagId;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        CarouselSubsystem1.OuttakePattern selectedPattern;

        // 1. Decidem pattern-ul pe baza AprilTag-ului
        switch (tagId) {
            case 21: selectedPattern = CarouselSubsystem1.OuttakePattern.GPP; break;
            case 22: selectedPattern = CarouselSubsystem1.OuttakePattern.PGP; break;
            case 23: selectedPattern = CarouselSubsystem1.OuttakePattern.PPG; break;
            default: selectedPattern = CarouselSubsystem1.OuttakePattern.GPP; break;
        }

        // 2. SALVĂM pattern-ul în subsistem (pentru colectările viitoare din timpul meciului)
        carousel.setActivePattern(selectedPattern);

        // 3. EXECUTĂM pregătirea imediată (pentru bilele deja aflate în carusel)
        carousel.prepareOuttake(selectedPattern);
    }

    @Override
    public boolean isFinished() {
        // Se termină instantaneu, lăsând FSM-ul din subsistem să rotească motorul în fundal
        return true;
    }
}