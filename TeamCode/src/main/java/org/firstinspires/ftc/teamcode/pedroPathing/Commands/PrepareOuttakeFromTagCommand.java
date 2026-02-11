package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import java.util.function.IntSupplier; // Importăm IntSupplier

public class PrepareOuttakeFromTagCommand extends CommandBase {
    private final CarouselSubsystem1 carousel;
    private final IntSupplier tagIdSupplier; // Stocăm o funcție, nu o valoare

    // Constructorul acceptă acum un IntSupplier
    public PrepareOuttakeFromTagCommand(CarouselSubsystem1 carousel, IntSupplier tagIdSupplier) {
        this.carousel = carousel;
        this.tagIdSupplier = tagIdSupplier;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        // Obținem ID-ul ACTUALizat la momentul execuției
        int currentTagId = tagIdSupplier.getAsInt();

        CarouselSubsystem1.OuttakePattern selectedPattern;

        // Logica switch, acum cu ID-urile corecte pentru randomizare
        switch (currentTagId) {
            case 21: // Poziția 1
                selectedPattern = CarouselSubsystem1.OuttakePattern.GPP;
                break;
            case 22: // Poziția 2
                selectedPattern = CarouselSubsystem1.OuttakePattern.PGP;
                break;
            case 23: // Poziția 3
                selectedPattern = CarouselSubsystem1.OuttakePattern.PPG;
                break;
            default: // Dacă nu vedem nimic (tagId = -1), folosim un default sigur
                selectedPattern = CarouselSubsystem1.OuttakePattern.GPP; // Sau ce consideri tu default
                break;
        }

        // Restul logicii rămâne la fel
        carousel.setActivePattern(selectedPattern);
        carousel.prepareOuttake(selectedPattern);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
