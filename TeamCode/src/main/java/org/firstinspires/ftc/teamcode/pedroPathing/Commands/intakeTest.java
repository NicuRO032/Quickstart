package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.CarouselSubsystem1;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.IntakeSubsystem1;

public class intakeTest extends SequentialCommandGroup {


    public intakeTest (CarouselSubsystem1 carousel, IntakeSubsystem1 intake){
        addCommands(
                new InstantCommand(() -> {
                    carousel.activateIntake();
                    carousel.setCarouselForIntake();
                    intake.collect();
                }),

                new WaitUntilCommand(carousel::allSlotsOccupied),


                 new InstantCommand(() -> {
                   if (carousel.allSlotsOccupied()) {
                       intake.eject();
                   }
/*
                   else if (carousel.getOccupied(2)){
                       intake.cleanup();
                   } */
        })

        );
        addRequirements(carousel, intake);
    }
}
