package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem;

import java.util.function.IntSupplier;

public class StartOuttakeFromStoredTagCommand extends CommandBase {

    private final CarouselSubsystem carousel;
    private final IntSupplier tagSupplier;

    private boolean started = false;

    public StartOuttakeFromStoredTagCommand(
            CarouselSubsystem carousel,
            IntSupplier tagSupplier
    ) {
        this.carousel = carousel;
        this.tagSupplier = tagSupplier;

        addRequirements(carousel);
    }

    @Override
    public void execute() {
        if (started) return;

        int tagId = tagSupplier.getAsInt();

        switch (tagId) {
            case 21:
                carousel.startOuttake(CarouselSubsystem.OuttakePattern.PGG);
                break;

            case 22:
                carousel.startOuttake(CarouselSubsystem.OuttakePattern.GPG);
                break;

            case 23:
                carousel.startOuttake(CarouselSubsystem.OuttakePattern.GGP);
                break;

            default:
                // fallback sigur
                carousel.startOuttake(CarouselSubsystem.OuttakePattern.PGG);
                break;
        }

        started = true;
    }

    @Override
    public boolean isFinished() {
        // comanda se termină DOAR când FSM-ul chiar a pornit
        return started && !carousel.getOuttakeState().equals("OUT_IDLE");
    }
}
