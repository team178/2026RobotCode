package frc.robot.autos;

/**
 * Represents a single mechanism action to be triggered at a specific waypoint
 * during an autonomous trajectory.
 *
 * @param waypointIndex The 0-based index of the waypoint in the trajectory at
 *                      which this action fires. The runner resolves this to an
 *                      actual timestamp at runtime using splits() + samples().
 * @param action        The action to execute (e.g. intake::deploy, shooter::shoot).
 *                      Use a lambda or method reference.
 */
public record MechanismTrigger(int waypointIndex, Runnable action) {}