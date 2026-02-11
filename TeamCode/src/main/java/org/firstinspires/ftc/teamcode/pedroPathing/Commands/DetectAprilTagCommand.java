package org.firstinspires.ftc.teamcode.pedroPathing.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.VisionSubsystem;
import java.util.function.Consumer;

public class DetectAprilTagCommand extends CommandBase {

    private final VisionSubsystem vision;
    private final Consumer<Integer> tagConsumer;
    private final long timeoutMs;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean tagFound = false;

    // NOU: ID-ul de fallback este acum o constantă internă.
    private static final int FALLBACK_TAG_ID = 21;

    /**
     * Comandă care scanează după un AprilTag pentru o perioadă limitată de timp.
     * Folosește un ID de fallback intern (21) dacă detecția eșuează.
     * @param vision Subsistemul de viziune.
     * @param tagConsumer Funcție pentru a stoca ID-ul găsit.
     * @param timeoutMs Timpul maxim de scanare, în milisecunde.
     */
    public DetectAprilTagCommand(VisionSubsystem vision, Consumer<Integer> tagConsumer, long timeoutMs) {
        this.vision = vision;
        this.tagConsumer = tagConsumer;
        this.timeoutMs = timeoutMs;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.enableProcesor();
        timer.reset();
        tagFound = false;
    }

    @Override
    public void execute() {
        if (!tagFound) {
            int tagId = vision.getLastTagId();
            if (tagId == 21 || tagId == 22 || tagId == 23) {
                tagConsumer.accept(tagId);
                tagFound = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return tagFound || timer.milliseconds() > timeoutMs;
    }

    @Override
    public void end(boolean interrupted) {
        if (!tagFound) {
            // Dacă a expirat timpul, folosim valoarea de fallback hardcodată.
            tagConsumer.accept(FALLBACK_TAG_ID);
        }
        //vision.disableProcesor();
    }
}
