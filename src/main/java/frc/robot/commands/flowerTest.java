
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.FrameConstants;
// import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Locomotion.Drive;
import frc.robot.subsystems.ultilities.FollowPathCommand103;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.pathfinding.Pathfinder;
// import com.pathplanner.lib.pathfinding.Pathfinding;
// import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// 
import com.pathplanner.lib.path.Waypoint;
// import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Random;

public class flowerTest extends Command {
  // -------------------- CONFIG --------------------
  private static final int NUM_PATHS = 6;

  // GA hyperparameters (start conservative)
  private static final int POP_SIZE = 6;
  private static final int ELITES = 3;
  private static final int GENERATIONS = 15;
  private static final double MUTATION_RATE = 0.30;
  private static final double MUTATION_SIGMA = 0.20;  // percentage of range

  // Reasonable bounds for FRC PID (tune if needed)
  private static final double P_MIN_T = 0.0,  P_MAX_T = 10.0;
  private static final double I_MIN_T = 0.0,  I_MAX_T =  .3;
  private static final double D_MIN_T = 0.0,  D_MAX_T =  1.5;

  private static final double P_MIN_R = 0.0,  P_MAX_R = 10.0;
  private static final double I_MIN_R = 0.0,  I_MAX_R =  1;
  private static final double D_MIN_R = 0.0,  D_MAX_R =  1.5;
  
  private static final double ALLOWABLE_COST = 2000.0; // if exceeded, abort path early
  private static final double HIGH_COST_PENALTY = 1e6; // cost assigned to failed runs
  private boolean HIGH_COST_FLAG = false;


  // -------------------- STATE --------------------
  private final List<PathPlannerPath> paths = new ArrayList<>();
  private final Drive drive;
  private int pathIndex = 0;
  private FollowPathCommand103 activeCmd;
  private Command homeCommand;
  private boolean running = false;

  // Tunable controller (Optional provided to your builder)
  private Optional<TunableHolonomicController> tunableController = Optional.empty();

  // Genome = {transP, transI, transD, rotP, rotI, rotD}
  private static class Genome {
    double tP, tI, tD, rP, rI, rD;
    Genome(double tP, double tI, double tD, double rP, double rI, double rD) {
      this.tP = tP; this.tI = tI; this.tD = tD;
      this.rP = rP; this.rI = rI; this.rD = rD;
    }
    Genome copy() { return new Genome(tP, tI, tD, rP, rI, rD); }
  }

  private static class Individual {
    Genome g;
    double fitness = Double.POSITIVE_INFINITY; // lower is better (cost)
    Individual(Genome g) { this.g = g; }
  }

  private final Random rng = new Random(2025);

  private List<Individual> population = new ArrayList<>();
  private int generation = 0;
  private int indIdx = 0;
  private double accumCostForIndividual = 0.0;

  // ------------- INITIAL GA SEED -------------
  // your current defaults as a starting point
  private double seedTP = 5.0, seedTI = 0.0, seedTD = 0.0;
  private double seedRP = 5.0, seedRI = 0.0, seedRD = 0.0;

  public flowerTest(Drive drive) {
    this.drive = drive;
    // resetPidPreferencesToDefaults();
    loadSeedsFromPreferences();

    // Controller instance (shared, but re-tuned per run)
    tunableController = Optional.of(new TunableHolonomicController(
        seedTP, seedTI, seedTD,
        seedTP, seedTI, seedTD,
        seedRP, seedRI, seedRD,
        FrameConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, FrameConstants.kphysicalMaxAngularAccelerationRadiansPerSecond));

    try {
      paths.add(PathPlannerPath.fromPathFile("W_1"));
      paths.add(PathPlannerPath.fromPathFile("W_2"));
      paths.add(PathPlannerPath.fromPathFile("W_3"));
      paths.add(PathPlannerPath.fromPathFile("W_4"));
      paths.add(PathPlannerPath.fromPathFile("W_5"));
      paths.add(PathPlannerPath.fromPathFile("W_6"));
      // paths.add(PathPlannerPath.fromPathFile("PIDTuning2"));
      // paths.add(PathPlannerPath.fromPathFile("PIDTuning2"));
      // paths.add(PathPlannerPath.fromPathFile("PID_1"));
      // paths.add(PathPlannerPath.fromPathFile("PID_2"));

      if (paths.size() != NUM_PATHS) {
        DriverStation.reportWarning("Expected " + NUM_PATHS + " paths; got " + paths.size(), false);
      }
    } catch (Exception e) {
      DriverStation.reportError("Error loading path: " + e.getMessage(), e.getStackTrace());
    }

    // No subsystem requirements shown; add if your Drive is a requirement:
    // addRequirements(drive);
  }

  // -------------------- PUBLIC CONTROL --------------------
  public void setRunning(boolean run) {
    running = run;
  }

  // -------------------- COMMAND LIFECYCLE --------------------
  @Override
  public void initialize() {
    // Initialize GA population
    initPopulation();
    generation = 0;
    indIdx = 0;
    pathIndex = 0;
    accumCostForIndividual = 0.0;

    // Kick off first evaluation run
    applyGenome(population.get(indIdx).g);
    schedulePath(pathIndex);
  }

  @Override
public void execute() {
  if (!running || activeCmd == null) return;

  if (!activeCmd.isFinished()) {
    double cost = activeCmd.getCostFunctionAccumulator();
    if (cost > ALLOWABLE_COST) {
      // Abort this path immediately
      activeCmd.cancel();
      drive.stopModules();                 // zero outputs now
      HIGH_COST_FLAG = true;
      pathIndex = NUM_PATHS - 1;    // fast-forward to end for this individual
      return;                       // wait for next tick to process end-of-path bookkeeping
    }
    return; // still running normally
  }

  // From here on, the command has finished (or was canceled in a prior tick)
  if (homeCommand == null || !homeCommand.isScheduled() || homeCommand.isFinished()) {
    double cost = activeCmd.getCostFunctionAccumulator();

    accumCostForIndividual += cost;
    if (HIGH_COST_FLAG) {
      accumCostForIndividual += HIGH_COST_PENALTY;
      HIGH_COST_FLAG = false;
    }

    pathIndex++;

    if (pathIndex < NUM_PATHS) {
      schedulePath(pathIndex);
      return;
    }

    if (pathIndex == NUM_PATHS) {
        
      // Finished evaluating this individual
      population.get(indIdx).fitness = accumCostForIndividual;
      var g = population.get(indIdx).g;

      SmartDashboard.putNumber("PID/transP", g.tP);
      SmartDashboard.putNumber("PID/transI", g.tI);
      SmartDashboard.putNumber("PID/transD", g.tD);
      SmartDashboard.putNumber("PID/rotP",   g.rP);
      SmartDashboard.putNumber("PID/rotI",   g.rI);
      SmartDashboard.putNumber("PID/rotD",   g.rD);
      SmartDashboard.putNumber("PID/cost",   population.get(indIdx).fitness);

      System.out.printf("\n[GA] Fitness=%.4f (sum of %d paths)\n", population.get(indIdx).fitness, NUM_PATHS);
      SmartDashboard.putNumber("individualCost", cost);
      SmartDashboard.putNumber("individualIndex", indIdx);

      // Reset to home
    //   Rotation2d homeRotation = paths.get(0).getGoalEndState().rotation();
      Pose2d homePoint = paths.get(0).getPathPoses().get(0);
      Pose2d startPoint = drive.getPose();
      if(homePoint.minus(startPoint).getTranslation().getNorm() > 0.5){  
        List<Pose2d> poses = new ArrayList<>();
        poses.add(startPoint);
        poses.add(homePoint);
        // List<Waypoint> pts = PathPlannerPath.waypointsFromPoses(poses);
        // PathPlannerPath homepath = new PathPlannerPath(
        //         pts,
        //         new PathConstraints(1.5, 1.5, 1.5, 1.5),
        //         new IdealStartingState(0, startPoint.getRotation()),
        //         new GoalEndState(0, homeRotation)
        //     );

        // TunableHolonomicController tempController = new TunableHolonomicController(
        //     5, 0, 0, 5, 0, 0, 5, 0, 0, Math.toRadians(720), Math.toRadians(720));
        PathConstraints constraints = new PathConstraints(
        2, 1.0,
           Math.PI/2, Math.PI/2);

        // homeCommand = AutoBuilder.pathfindToPose(homePoint, constraints,0.0);
        // // homeCommand = drive.PathCommandBuilder103(homepath, Optional.of(tempController))
        // //                     .finallyDo(i -> drive.stopModules());  // <-- ensure stop on finish/cancel
        // homeCommand.schedule();
        }
      return;
    }

    // Move to next individual
    indIdx++;
    if (indIdx < population.size()) {
      pathIndex = 0;
      accumCostForIndividual = 0.0;
      applyGenome(population.get(indIdx).g);
      schedulePath(pathIndex);
    } else {
      saveBestToPreferences();
      evolvePopulation();
      generation++;
      if (generation >= GENERATIONS) {
        running = false;
        System.out.printf("[GA] Completed %d generations. Best saved to Preferences.", GENERATIONS);
        return;
      }
      indIdx = 0;
      pathIndex = 0;
      accumCostForIndividual = 0.0;
      applyGenome(population.get(indIdx).g);
      schedulePath(pathIndex);
    }
  }
}

  @Override
  public void end(boolean interrupted) {
    if (interrupted && activeCmd != null && activeCmd.isScheduled()) {
      activeCmd.cancel();
      saveBestToPreferences();
    }
  }

  // -------------------- EVALUATION --------------------
  private void schedulePath(int idx) {
    if (idx < 0 || idx >= paths.size()) {
      DriverStation.reportError("[GA] Bad path index " + idx, false);
      return;
    }
    activeCmd = drive.PathCommandBuilder103(paths.get(idx), tunableController);
    activeCmd.schedule();
  }

  private void applyGenome(Genome g) {
    // Push gains into the controller before scheduling any path
    tunableController.ifPresent(ctrl -> {
      ctrl.setTranslationGains(g.tP, g.tI, g.tD);
      ctrl.setRotationGains(g.rP, g.rI, g.rD);
    });
  }

  // -------------------- GA CORE --------------------
  private void initPopulation() {
    population.clear();
    // Elites seeded around your current values + randoms
    population.add(new Individual(new Genome(seedTP, seedTI, seedTD, seedRP, seedRI, seedRD)));
    population.add(new Individual(jitterAround(seedTP, seedTI, seedTD, seedRP, seedRI, seedRD, 0.15)));
    population.add(new Individual(jitterAround(seedTP, seedTI, seedTD, seedRP, seedRI, seedRD, 0.15)));

    // Fill remainder with random genomes within bounds
    while (population.size() < POP_SIZE) {
      population.add(new Individual(randomGenome()));
    }
  }

  private Genome jitterAround(double tP, double tI, double tD, double rP, double rI, double rD, double frac) {
    return new Genome(
        clamp(tP * (1 + randSym(frac)), P_MIN_T, P_MAX_T),
        clamp(tI * (1 + randSym(frac)), I_MIN_T, I_MAX_T),
        clamp(tD * (1 + randSym(frac)), D_MIN_T, D_MAX_T),
        clamp(rP * (1 + randSym(frac)), P_MIN_R, P_MAX_R),
        clamp(rI * (1 + randSym(frac)), I_MIN_R, I_MAX_R),
        clamp(rD * (1 + randSym(frac)), D_MIN_R, D_MAX_R)
    );
  }

  private Genome randomGenome() {
    return new Genome(
        randIn(P_MIN_T, P_MAX_T),
        randIn(I_MIN_T, I_MAX_T),
        randIn(D_MIN_T, D_MAX_T),
        randIn(P_MIN_R, P_MAX_R),
        randIn(I_MIN_R, I_MAX_R),
        randIn(D_MIN_R, D_MAX_R)
    );
  }

  private void evolvePopulation() {
    // Sort by fitness (lower is better)
    population.sort((a, b) -> Double.compare(a.fitness, b.fitness));

    // Keep a copy of the best individual for reporting
    Individual best = population.get(0);
    DriverStation.reportWarning(String.format(
        "[GA] Gen %d best cost=%.5f  tP=%.3f tI=%.3f tD=%.3f  rP=%.3f rI=%.3f rD=%.3f",
        generation, best.fitness,
        best.g.tP, best.g.tI, best.g.tD, best.g.rP, best.g.rI, best.g.rD),
        false);

    // Elitism
    List<Individual> nextPop = new ArrayList<>();
    for (int i = 0; i < ELITES; i++) {
      nextPop.add(new Individual(population.get(i).g.copy()));
    }

    // Fill remainder via tournament selection + crossover + mutation
    while (nextPop.size() < POP_SIZE) {
      Individual p1 = tournamentSelect(3);
      Individual p2 = tournamentSelect(3);
      Genome c = blendCrossover(p1.g, p2.g, 0.3);
      mutate(c);
      nextPop.add(new Individual(c));
    }

    population = nextPop;
    // Reset fitness for next generation
    for (var ind : population) ind.fitness = Double.POSITIVE_INFINITY;
  }

  private Individual tournamentSelect(int k) {
    Individual best = null;
    for (int i = 0; i < k; i++) {
      Individual cand = population.get(rng.nextInt(population.size()));
      if (best == null || cand.fitness < best.fitness) best = cand;
    }
    return best;
  }

  // BLX-alpha / blend-like crossover between two numbers
  private double blend(double a, double b, double alpha, double min, double max) {
    double low = Math.min(a, b);
    double high = Math.max(a, b);
    double range = high - low;
    double lo = low - alpha * range;
    double hi = high + alpha * range;
    return clamp(randIn(lo, hi), min, max);
  }

  private Genome blendCrossover(Genome g1, Genome g2, double alpha) {
    return new Genome(
        blend(g1.tP, g2.tP, alpha, P_MIN_T, P_MAX_T),
        blend(g1.tI, g2.tI, alpha, I_MIN_T, I_MAX_T),
        blend(g1.tD, g2.tD, alpha, D_MIN_T, D_MAX_T),
        blend(g1.rP, g2.rP, alpha, P_MIN_R, P_MAX_R),
        blend(g1.rI, g2.rI, alpha, I_MIN_R, I_MAX_R),
        blend(g1.rD, g2.rD, alpha, D_MIN_R, D_MAX_R)
    );
  }

  private void mutate(Genome g) {
    // Gaussian mutation applied per gene with probability
    if (rng.nextDouble() < MUTATION_RATE) g.tP = mutateGaussian(g.tP, P_MIN_T, P_MAX_T);
    if (rng.nextDouble() < MUTATION_RATE) g.tI = mutateGaussian(g.tI, I_MIN_T, I_MAX_T);
    if (rng.nextDouble() < MUTATION_RATE) g.tD = mutateGaussian(g.tD, D_MIN_T, D_MAX_T);
    if (rng.nextDouble() < MUTATION_RATE) g.rP = mutateGaussian(g.rP, P_MIN_R, P_MAX_R);
    if (rng.nextDouble() < MUTATION_RATE) g.rI = mutateGaussian(g.rI, I_MIN_R, I_MAX_R);
    if (rng.nextDouble() < MUTATION_RATE) g.rD = mutateGaussian(g.rD, D_MIN_R, D_MAX_R);
  }

  private double mutateGaussian(double val, double min, double max) {
    double range = max - min;
    double sigma = MUTATION_SIGMA * range;
    double mutated = val + rng.nextGaussian() * sigma;
    return clamp(mutated, min, max);
  }

  // -------------------- PERSIST BEST --------------------
  private void saveBestToPreferences() {
    if (population == null || population.isEmpty()) return;

    var best = population.stream()
        .filter(ind -> !Double.isNaN(ind.fitness))
        .min(Comparator.comparingDouble(ind -> ind.fitness))   // lower is better
        .orElse(population.get(0)); // fallback
  
    Genome g = best.g;
    Preferences.setDouble("PID/transP", g.tP);
    Preferences.setDouble("PID/transI", g.tI);
    Preferences.setDouble("PID/transD", g.tD);
    Preferences.setDouble("PID/rotP", g.rP);
    Preferences.setDouble("PID/rotI", g.rI);
    Preferences.setDouble("PID/rotD", g.rD);
  }
@SuppressWarnings("unused")
private void DisplayBestPID() {
  if (population == null || population.isEmpty()) return;

  var best = population.stream()
      .filter(ind -> !Double.isNaN(ind.fitness))
      .min(Comparator.comparingDouble(ind -> ind.fitness))   // lower is better
      .orElse(population.get(0)); // fallback

  Genome g = best.g;

  SmartDashboard.putNumber("PID/transP", g.tP);
  SmartDashboard.putNumber("PID/transI", g.tI);
  SmartDashboard.putNumber("PID/transD", g.tD);
  SmartDashboard.putNumber("PID/rotP",   g.rP);
  SmartDashboard.putNumber("PID/rotI",   g.rI);
  SmartDashboard.putNumber("PID/rotD",   g.rD);
  SmartDashboard.putNumber("PID/cost",   best.fitness);
}


  // -------------------- UTILS --------------------
  private double clamp(double v, double min, double max) {
    return Math.max(min, Math.min(max, v));
  }
  private double randIn(double min, double max) {
    return min + rng.nextDouble() * (max - min);
  }
  private double randSym(double magnitude) {
    return (rng.nextDouble() * 2 - 1) * magnitude; // [-mag, +mag]
  }
  
    private void loadSeedsFromPreferences() {
    // If keys don't exist yet, publish the current defaults so you can see them
        if (!Preferences.containsKey("PID/transP")) Preferences.setDouble("PID/transP", seedTP);
        if (!Preferences.containsKey("PID/transI")) Preferences.setDouble("PID/transI", seedTI);
        if (!Preferences.containsKey("PID/transD")) Preferences.setDouble("PID/transD", seedTD);
        if (!Preferences.containsKey("PID/rotP"))   Preferences.setDouble("PID/rotP",  seedRP);
        if (!Preferences.containsKey("PID/rotI"))   Preferences.setDouble("PID/rotI",  seedRI);
        if (!Preferences.containsKey("PID/rotD"))   Preferences.setDouble("PID/rotD",  seedRD);
    
        // Read back (with current seed as fallback)
        seedTP = sanitize(Preferences.getDouble("PID/transP", seedTP), 0.0, 18.0);
        seedTI = sanitize(Preferences.getDouble("PID/transI", seedTI), 0.0,  1.0);
        seedTD = sanitize(Preferences.getDouble("PID/transD", seedTD), 0.0,  2.5);
    
        seedRP = sanitize(Preferences.getDouble("PID/rotP",  seedRP), 0.0, 18.0);
        seedRI = sanitize(Preferences.getDouble("PID/rotI",  seedRI), 0.0,  1.0);
        seedRD = sanitize(Preferences.getDouble("PID/rotD",  seedRD), 0.0,  3.0);
    }
    
  private double sanitize(double v, double min, double max) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return min;
        return Math.max(min, Math.min(max, v));
    }
  
public static void resetPidPreferencesToDefaults() {
    Preferences.setDouble("PID/transP", 5.0);
    Preferences.setDouble("PID/transI", 0.0);
    Preferences.setDouble("PID/transD", 0.0);
    Preferences.setDouble("PID/rotP",   5.0);
    Preferences.setDouble("PID/rotI",   0.0);
    Preferences.setDouble("PID/rotD",   0.0);
  }
  
}
