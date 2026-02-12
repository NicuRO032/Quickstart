package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;

// În ShootAllBallsCommand.java
public class ShootAllBallsCommand extends SequentialCommandGroup {

    public static final long SHOOTER_TIMEOUT_MS = 3000; // Mărește timeout-ul la 3s pentru siguranță

    public ShootAllBallsCommand(CarouselSubsystem1 carousel) {
        addCommands(
                // Așteaptă până când caruselul e gata de tragere SAU trece timeout-ul
                new WaitUntilCommand(carousel::isReadyToShoot).withTimeout(SHOOTER_TIMEOUT_MS),

                // Comandă tragerea
                new InstantCommand(carousel::triggerShoot),

                // Așteaptă până când ciclul de outtake se termină complet (revine la IDLE)
                // MODIFICAT: Comparație directă cu enum-ul pentru siguranță
                new WaitUntilCommand(() -> carousel.getOuttakeStateEnum() == CarouselSubsystem1.OuttakeState.OUT_IDLE)
        );
        addRequirements(carousel);
    }
}
