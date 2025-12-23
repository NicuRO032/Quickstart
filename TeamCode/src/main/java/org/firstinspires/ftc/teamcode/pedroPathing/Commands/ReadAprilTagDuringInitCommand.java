package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;

import java.util.function.IntConsumer;

public class ReadAprilTagDuringInitCommand extends CommandBase {

    private final VisionSubsystem vision;
    private final IntConsumer store;

    public ReadAprilTagDuringInitCommand(
            VisionSubsystem vision,
            IntConsumer store
    ) {
        this.vision = vision;
        this.store = store;
    }

    @Override
    public void execute() {
        if (vision.hasValidTag()) {
            store.accept(vision.getLastTagId());
        }
    }

    @Override
    public boolean isFinished() {
        // NU blocăm INIT
        return false;
    }
}