package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;

public class ShootAllBallsCommand extends SequentialCommandGroup {

    public ShootAllBallsCommand(CarouselSubsystem1 carousel) {
        addCommands(
                // 1. Așteptăm ca carouselul să ajungă la prima bilă și shooter-ul să fie gata
                new WaitUntilCommand(carousel::isReadyToShoot),

                // 2. Declanșăm prima tragere (FSM-ul va face restul automat)
                new InstantCommand(carousel::triggerShoot),

                // 3. Așteptăm până când FSM-ul termină toate bilele și revine în IDLE
                new WaitUntilCommand(() -> carousel.getOuttakeState().equals("OUT_IDLE"))
        );

        // Adăugăm subsistemul ca cerință pentru a preveni alte comenzi să intervină
        addRequirements(carousel);
    }
}