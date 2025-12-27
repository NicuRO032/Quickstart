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

        // Maparea ID-ului tag-ului la pattern-ul dorit
        switch (tagId) {
            case 21:
                selectedPattern = CarouselSubsystem1.OuttakePattern.PGG;
                break;
            case 22:
                selectedPattern = CarouselSubsystem1.OuttakePattern.GPG;
                break;
            case 23:
                selectedPattern = CarouselSubsystem1.OuttakePattern.GGP;
                break;
            default:
                // Fallback în cazul în care tag-ul nu a fost detectat corect
                selectedPattern = CarouselSubsystem1.OuttakePattern.PGG;
                break;
        }

        // Apelăm logica de pregătire din subsistem
        // Aceasta va seta motorul de shooter și va roti carouselul spre prima bilă
        carousel.prepareOuttake(selectedPattern);
    }

    @Override
    public boolean isFinished() {
        return true; // Comanda se termină imediat ce a trimis instrucțiunea
    }
}