package frc.robot.autos;

import java.util.List;

/**
 * Immutable definition of a single autonomous strand.
 *
 * <p>A strand is one segment of a full autonomous sequence: one Choreo trajectory
 * file paired with the mechanism triggers that fire during it.
 *
 * @param trajName       The exact name of the .traj file (without extension) as it
 *                       appears in the deploy/choreo directory. E.g. "Auto1__1_2".
 * @param resetOdometry  True only for the very first strand in a sequence (i.e. the
 *                       strand that starts from a known Starting Position). All
 *                       subsequent strands must be false to maintain odometry
 *                       continuity.
 * @param triggers       Ordered list of mechanism triggers to register on this
 *                       trajectory. Each trigger fires once at the timestamp of its
 *                       declared waypoint index.
 */
public record Stranddefinition(
        String trajName,
        boolean resetOdometry,
        List<MechanismTrigger> triggers) {

    /**
     * Convenience constructor for strands with no mechanism triggers.
     */
    public Stranddefinition(String trajName, boolean resetOdometry) {
        this(trajName, resetOdometry, List.of());
    }
}