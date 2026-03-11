package frc.robot.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

/**
 * Converts a {@link StrandDefinition} into a runnable WPILib {@link Command}.
 *
 * <p>For each strand:
 * <ol>
 *   <li>A fresh {@link AutoRoutine} is created so its EventLoop is isolated.</li>
 *   <li>The named trajectory is loaded from the deploy directory.</li>
 *   <li>Waypoint timestamps are resolved at runtime via
 *       {@code getRawTrajectory().splits()} + {@code .samples().get(idx).getTimestamp()}.
 *       No timestamps are hardcoded anywhere.</li>
 *   <li>Each {@link MechanismTrigger} is registered with {@code atTime(seconds)}
 *       so it fires exactly once when the trajectory reaches that waypoint.</li>
 *   <li>If {@code resetOdometry} is true, an odometry-reset command is prepended.</li>
 * </ol>
 *
 * <p>The returned command is a self-contained {@link AutoRoutine#cmd()} that polls
 * its own EventLoop for the duration of the trajectory.
 */
public class StrandRunner {

    private final AutoFactory autoFactory;

    /**
     * @param autoFactory The shared AutoFactory from RobotContainer, already
     *                    configured with pose supplier, odometry reset consumer,
     *                    and follow-trajectory consumer.
     */
    public StrandRunner(AutoFactory autoFactory) {
        this.autoFactory = autoFactory;
    }

    /**
     * Builds a fully configured {@link Command} for a single strand.
     *
     * @param def The strand definition (trajectory name + odometry flag + triggers).
     * @return A command that, when scheduled, follows the trajectory and fires all
     *         mechanism triggers at their respective waypoints.
     */
    public Command buildStrand(Stranddefinition def) {
        // Each strand gets its own routine so EventLoops don't bleed across strands.
        AutoRoutine routine = autoFactory.newRoutine(def.trajName());
        AutoTrajectory traj = routine.trajectory(def.trajName());

        // ---------------------------------------------------------------
        // Resolve waypoint timestamps at runtime.
        // splits() returns List<Integer> where each element is the sample
        // index of that waypoint. samples().get(idx).getTimestamp() gives
        // the elapsed time in seconds.
        // ---------------------------------------------------------------
        Trajectory<SwerveSample> rawTraj = traj.getRawTrajectory();
        List<Integer> splitIndices = rawTraj.splits();
        List<SwerveSample> samples = rawTraj.samples();

        double[] waypointTimes = new double[splitIndices.size()];
        for (int i = 0; i < splitIndices.size(); i++) {
            int sampleIdx = splitIndices.get(i);
            // Guard: clamp to valid range in case of malformed trajectory
            sampleIdx = Math.min(sampleIdx, samples.size() - 1);
            waypointTimes[i] = samples.get(sampleIdx).getTimestamp();
        }

        // ---------------------------------------------------------------
        // Register mechanism triggers.
        // Each trigger fires ONCE (atTime produces a rising-edge trigger)
        // at the resolved waypoint timestamp.
        // ---------------------------------------------------------------
        for (MechanismTrigger trigger : def.triggers()) {
            int wpIdx = trigger.waypointIndex();
            if (wpIdx < 0 || wpIdx >= waypointTimes.length) {
                System.err.println("[StrandRunner] WARNING: waypoint index " + wpIdx
                        + " out of range for trajectory " + def.trajName()
                        + " (has " + waypointTimes.length + " waypoints). Trigger skipped.");
                continue;
            }
            double triggerTime = waypointTimes[wpIdx];
            traj.atTime(triggerTime).onTrue(
                    Commands.runOnce(trigger.action())
            );
        }

        // ---------------------------------------------------------------
        // Wire the routine: start following the trajectory when the
        // routine becomes active (standard Choreo pattern).
        // ---------------------------------------------------------------
        routine.active().onTrue(traj.cmd());

        // ---------------------------------------------------------------
        // Odometry reset: prepend a reset command only for the first strand.
        // ---------------------------------------------------------------
        Command routineCmd = routine.cmd();

        if (def.resetOdometry()) {
            Command resetCmd = autoFactory.resetOdometry(def.trajName());
            return Commands.sequence(resetCmd, routineCmd);
        }

        return routineCmd;
    }
}