package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem;

public class AutoStoreCommand extends CommandBase {
    private final CarouselSubsystem carousel;
    private boolean ballHandled = false;
    private int stepsTried = 0;

    public AutoStoreCommand(CarouselSubsystem carousel) {
        this.carousel = carousel;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        ballHandled = false;
        stepsTried = 0;
    }

    @Override
    public void execute() {

        // așteptăm apariția bilei în slotul de intrare
        if (!carousel.entrySlotHasBall()) {
            return;
        }

        // bila tocmai a ajuns → o marcăm o singură dată
        if (!ballHandled) {
            carousel.markCurrentSlotOccupied();
            ballHandled = true;
        }

        // dacă toate sunt ocupate → ne oprim
        if (carousel.allSlotsOccupied()) {
            return;
        }

        // dacă slotul curent e ocupat → căutăm altul
        if (carousel.atTarget()) {
            carousel.stepRight();
            stepsTried++;
        }
    }

    @Override
    public boolean isFinished() {
        return ballHandled &&
                (carousel.isCurrentSlotFree()
                        || carousel.allSlotsOccupied()
                        || stepsTried >= 3);
    }
}
