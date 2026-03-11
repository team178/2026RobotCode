// package frc.robot.autos;

// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.ClimberSubsystem;

// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;

// /**
//  * Central registry of all 37 autonomous strand definitions.
//  *
//  * <p>Each entry maps a trajectory name (matching the .traj filename exactly) to a
//  * {@link Stranddefinition} containing:
//  * <ul>
//  *   <li>The trajectory name</li>
//  *   <li>Whether this strand resets odometry (true only when it is the FIRST strand
//  *       selected, i.e. starts from a known Starting Position "1")</li>
//  *   <li>A list of {@link MechanismTrigger}s — each specifying a 0-based waypoint
//  *       index and the Runnable to execute at that waypoint</li>
//  * </ul>
//  *
//  * <p><b>Waypoint Index Convention:</b> Indices match the PDF documentation exactly.
//  * WP0 = first waypoint (index 0), WP1 = second waypoint (index 1), etc.
//  * The StrandRunner resolves these to actual timestamps at runtime.
//  *
//  * <p><b>Odometry Reset Rule:</b> {@code resetOdometry} is stored as {@code false}
//  * for ALL strands here. The {@code StrandRunner} (or RobotContainer) sets it to
//  * {@code true} only on the first strand in the sequence at runtime. This avoids
//  * duplicating a boolean across all 37 definitions.
//  *
//  * <p><b>Intake Semantics:</b>
//  * <ul>
//  *   <li>{@code deploy()} — moves the intake arm DOWN only (no rollers)</li>
//  *   <li>{@code intake()} — runs intake rollers to collect game piece</li>
//  *   <li>{@code stop()}  — stops intake rollers</li>
//  * </ul>
//  * These are always called separately per the PDF instructions.
//  *
//  * <p><b>Shooter Duration:</b> The PDF specifies durations (e.g. "6 seconds").
//  * These are implemented as timed commands using {@code Commands.waitSeconds()} chained
//  * with a stop, scheduled via {@code Commands.sequence()} inside the trigger Runnable.
//  * The indexer is integrated into {@link ShooterSubsystem} — {@code shoot()} and
//  * {@code stop()} handle both mechanisms. The Runnable itself is fire-and-forget
//  * (schedules a command); it does not block the trajectory follow command.
//  */
// public class StrandRegistry {

//     private final Map<String, Stranddefinition> registry = new HashMap<>();

//     // -------------------------------------------------------------------------
//     // Subsystem references (injected via constructor)
//     // -------------------------------------------------------------------------
//     private final IntakeSubsystem intake;
//     private final ShooterSubsystem shooter;
//     private final ClimberSubsystem climber;

//     // Cached scheduler reference for timed stop commands
//     private final edu.wpi.first.wpilibj2.command.CommandScheduler scheduler =
//             edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance();

//     public StrandRegistry(
//             IntakeSubsystem intake,
//             ShooterSubsystem shooter,
//             ClimberSubsystem climber) {
//         this.intake = intake;
//         this.shooter = shooter;
//         this.climber = climber;
//         registerAll();
//     }

//     /**
//      * Returns the {@link Stranddefinition} for the given trajectory name.
//      *
//      * @param trajName Exact trajectory file name, e.g. "Auto1__2_3"
//      * @return The definition, or null if not found (logs a warning).
//      */
//     public Stranddefinition get(String trajName) {
//         Stranddefinition def = registry.get(trajName);
//         if (def == null) {
//             System.err.println("[StrandRegistry] WARNING: No definition found for '" + trajName + "'");
//         }
//         return def;
//     }

//     /**
//      * Returns a copy of the definition with {@code resetOdometry} forced to true.
//      * Called by RobotContainer/getAutonomousCommand() on the FIRST strand only.
//      */
//     public Stranddefinition getWithReset(String trajName) {
//         Stranddefinition def = get(trajName);
//         if (def == null) return null;
//         return new Stranddefinition(def.trajName(), true, def.triggers());
//     }

//     // =========================================================================
//     // HELPER METHODS for common trigger patterns
//     // =========================================================================

//     /** deploy() only — arm moves down, no rollers */
//     private MechanismTrigger deployAt(int wp) {
//         return new MechanismTrigger(wp, intake::deploy);
//     }

//     /** intake() rollers on */
//     private MechanismTrigger intakeOnAt(int wp) {
//         return new MechanismTrigger(wp, intake::intake);
//     }

//     /** intake stop() */
//     private MechanismTrigger intakeOffAt(int wp) {
//         return new MechanismTrigger(wp, intake::stop);
//     }

//     /**
//      * Shoots for {@code durationSeconds} then stops the shooter.
//      * The indexer is integrated into ShooterSubsystem — shoot() and stop()
//      * handle both mechanisms together.
//      * Schedules a timed command sequence; does not block the trajectory.
//      */
//     private MechanismTrigger shootAt(int wp, double durationSeconds) {
//         return new MechanismTrigger(wp, () -> {
//             scheduler.schedule(
//                 edu.wpi.first.wpilibj2.command.Commands.sequence(
//                     edu.wpi.first.wpilibj2.command.Commands.runOnce(shooter::shoot),
//                     edu.wpi.first.wpilibj2.command.Commands.waitSeconds(durationSeconds),
//                     edu.wpi.first.wpilibj2.command.Commands.runOnce(shooter::stop)
//                 )
//             );
//         });
//     }

//     /** Extend climber (raise arms above Rung 1) */
//     private MechanismTrigger extendAt(int wp) {
//         return new MechanismTrigger(wp, climber::extend);
//     }

//     /** Climb (retract — pulls robot off ground) */
//     private MechanismTrigger climbAt(int wp) {
//         return new MechanismTrigger(wp, climber::climb);
//     }

//     // =========================================================================
//     // REGISTRATION — all 37 strands
//     // resetOdometry is false for all; it is overridden at runtime for strand 1.
//     // =========================================================================

//     private void register(Stranddefinition def) {
//         registry.put(def.trajName(), def);
//     }

//     private void registerAll() {

//         // =====================================================================
//         // AUTO 1 — 12 strands
//         // =====================================================================

//         // Auto1__1_2
//         // WP1: deploy intake arm
//         register(new Stranddefinition("Auto1__1_2", false, List.of(
//                 deployAt(1)
//         )));

//         // Auto1__1_3
//         // WP1: deploy intake arm
//         // WP4: rollers on (entering depot slow creep)
//         // WP6: rollers off (at end / reversing out)
//         register(new Stranddefinition("Auto1__1_3", false, List.of(
//                 deployAt(1),
//                 intakeOnAt(4),
//                 intakeOffAt(6)
//         )));

//         // Auto1__1_7 (straight to climb)
//         // WP1: deploy intake arm
//         // WP4: extend climber
//         // WP5: climb
//         register(new Stranddefinition("Auto1__1_7", false, List.of(
//                 deployAt(1),
//                 extendAt(4),
//                 climbAt(5)
//         )));

//         // Auto1__1_Preload
//         // WP1: deploy intake arm
//         // WP3: shoot + index for 6 seconds
//         register(new Stranddefinition("Auto1__1_Preload", false, List.of(
//                 deployAt(1),
//                 shootAt(3, 6.0)
//         )));

//         // Auto1__2_3
//         // WP1: intake on  | WP3: intake off
//         // WP7: intake on  | WP9: intake off  (second pickup in depot)
//         register(new Stranddefinition("Auto1__2_3", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(3),
//                 intakeOnAt(7),
//                 intakeOffAt(9)
//         )));

//         // Auto1__2_4
//         // WP1: intake on  | WP3: intake off
//         // WP7: shoot + index for 8 seconds
//         register(new Stranddefinition("Auto1__2_4", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(3),
//                 shootAt(7, 8.0)
//         )));

//         // Auto1__2_5
//         // WP1: intake on  | WP4: intake off
//         register(new Stranddefinition("Auto1__2_5", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(4)
//         )));

//         // Auto1__3_4
//         // WP2: shoot + index for 8 seconds
//         register(new Stranddefinition("Auto1__3_4", false, List.of(
//                 shootAt(2, 8.0)
//         )));

//         // Auto1__4_3
//         // WP2: intake on  | WP4: intake off
//         register(new Stranddefinition("Auto1__4_3", false, List.of(
//                 intakeOnAt(2),
//                 intakeOffAt(4)
//         )));

//         // Auto1__4_7
//         // WP3: extend climber
//         // WP4: climb
//         register(new Stranddefinition("Auto1__4_7", false, List.of(
//                 extendAt(3),
//                 climbAt(4)
//         )));

//         // Auto1__5_6
//         // WP5: shoot + index for 6 seconds
//         register(new Stranddefinition("Auto1__5_6", false, List.of(
//                 shootAt(5, 6.0)
//         )));

//         // Auto1__6_8
//         // WP2: extend climber
//         // WP3: climb
//         register(new Stranddefinition("Auto1__6_8", false, List.of(
//                 extendAt(2),
//                 climbAt(3)
//         )));

//         // =====================================================================
//         // AUTO 2 — 17 strands
//         // =====================================================================

//         // Auto2__1_2
//         // WP1: deploy intake arm
//         register(new Stranddefinition("Auto2__1_2", false, List.of(
//                 deployAt(1)
//         )));

//         // Auto2__1_3
//         // WP1: deploy intake arm
//         // WP2–WP4: intake rollers on then off
//         register(new Stranddefinition("Auto2__1_3", false, List.of(
//                 deployAt(1),
//                 intakeOnAt(2),
//                 intakeOffAt(4)
//         )));

//         // Auto2__1_5
//         // WP1: deploy intake arm only
//         register(new Stranddefinition("Auto2__1_5", false, List.of(
//                 deployAt(1)
//         )));

//         // Auto2__1_7
//         // WP1: deploy intake arm
//         // WP3: extend climber
//         // WP4: climb
//         register(new Stranddefinition("Auto2__1_7", false, List.of(
//                 deployAt(1),
//                 extendAt(3),
//                 climbAt(4)
//         )));

//         // Auto2__1_8
//         // WP1: deploy intake arm
//         // WP3: extend climber
//         // WP4: climb
//         register(new Stranddefinition("Auto2__1_8", false, List.of(
//                 deployAt(1),
//                 extendAt(3),
//                 climbAt(4)
//         )));

//         // Auto2__1_Preload
//         // WP1: deploy intake arm
//         // WP2: shoot + index for 4 seconds
//         register(new Stranddefinition("Auto2__1_Preload", false, List.of(
//                 deployAt(1),
//                 shootAt(2, 4.0)
//         )));

//         // Auto2__2_3
//         // WP1–WP3: intake on/off
//         // WP7–WP9: intake on/off (second pickup)
//         register(new Stranddefinition("Auto2__2_3", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(3),
//                 intakeOnAt(7),
//                 intakeOffAt(9)
//         )));

//         // Auto2__2_4a
//         // WP1–WP3: intake on/off
//         // WP7: shoot + index for 8 seconds
//         register(new Stranddefinition("Auto2__2_4a", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(3),
//                 shootAt(7, 8.0)
//         )));

//         // Auto2__2_4b
//         // WP5: shoot + index for 8 seconds
//         register(new Stranddefinition("Auto2__2_4b", false, List.of(
//                 shootAt(5, 8.0)
//         )));

//         // Auto2__2_5
//         // WP1–WP4: intake on/off
//         register(new Stranddefinition("Auto2__2_5", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(4)
//         )));

//         // Auto2__3_4
//         // WP2: shoot + index for 8 seconds
//         register(new Stranddefinition("Auto2__3_4", false, List.of(
//                 shootAt(2, 8.0)
//         )));

//         // Auto2__4_3
//         // WP2–WP4: intake on/off
//         register(new Stranddefinition("Auto2__4_3", false, List.of(
//                 intakeOnAt(2),
//                 intakeOffAt(4)
//         )));

//         // Auto2__4_7
//         // WP3: extend climber
//         // WP4: climb
//         register(new Stranddefinition("Auto2__4_7", false, List.of(
//                 extendAt(3),
//                 climbAt(4)
//         )));

//         // Auto2__5_2
//         // WP1–WP4: intake on/off
//         register(new Stranddefinition("Auto2__5_2", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(4)
//         )));

//         // Auto2__5_6a
//         // WP1–WP3: intake on/off
//         // WP8: shoot + index for 8 seconds
//         register(new Stranddefinition("Auto2__5_6a", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(3),
//                 shootAt(8, 8.0)
//         )));

//         // Auto2__5_6b
//         // WP5: shoot + index for 6 seconds
//         register(new Stranddefinition("Auto2__5_6b", false, List.of(
//                 shootAt(5, 6.0)
//         )));

//         // Auto2__6_8
//         // WP2: extend climber
//         // WP3: climb
//         register(new Stranddefinition("Auto2__6_8", false, List.of(
//                 extendAt(2),
//                 climbAt(3)
//         )));

//         // =====================================================================
//         // AUTO 3 — 8 strands
//         // =====================================================================

//         // Auto3__1_5
//         // WP1: deploy intake arm only
//         register(new Stranddefinition("Auto3__1_5", false, List.of(
//                 deployAt(1)
//         )));

//         // Auto3__1_8
//         // WP1: deploy intake arm
//         // WP3: extend climber
//         // WP4: climb
//         register(new Stranddefinition("Auto3__1_8", false, List.of(
//                 deployAt(1),
//                 extendAt(3),
//                 climbAt(4)
//         )));

//         // Auto3__1_Preload
//         // WP1: deploy intake arm
//         // WP3: shoot + index for 4 seconds
//         register(new Stranddefinition("Auto3__1_Preload", false, List.of(
//                 deployAt(1),
//                 shootAt(3, 4.0)
//         )));

//         // Auto3__2_4
//         // WP5: shoot + index for 8 seconds
//         register(new Stranddefinition("Auto3__2_4", false, List.of(
//                 shootAt(5, 8.0)
//         )));

//         // Auto3__4_7
//         // WP3: extend climber
//         // WP4: climb
//         register(new Stranddefinition("Auto3__4_7", false, List.of(
//                 extendAt(3),
//                 climbAt(4)
//         )));

//         // Auto3__5_2
//         // WP1–WP4: intake on/off
//         register(new Stranddefinition("Auto3__5_2", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(4)
//         )));

//         // Auto3__5_6
//         // WP1–WP3: intake on/off
//         // WP8: shoot + index for 8 seconds
//         register(new Stranddefinition("Auto3__5_6", false, List.of(
//                 intakeOnAt(1),
//                 intakeOffAt(3),
//                 shootAt(8, 8.0)
//         )));

//         // Auto3__6_8
//         // WP2: extend climber
//         // WP3: climb
//         register(new Stranddefinition("Auto3__6_8", false, List.of(
//                 extendAt(2),
//                 climbAt(3)
//         )));
//     }
// }