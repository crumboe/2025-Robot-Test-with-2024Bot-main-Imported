
package frc.robot.commands;

/**
 * GENETIC ALGORITHM FOR PID TUNING
 * 
 * OVERVIEW:
 * This command implements a Genetic Algorithm (GA) to automatically optimize PID controller gains
 * for swerve drive path following. Instead of manually tuning 6 PID parameters (translational P/I/D
 * and rotational P/I/D), we use evolution to search the parameter space and find gains that minimize
 * tracking error across a set of test paths.
 * 
 * WHAT IS A GENETIC ALGORITHM?
 * A genetic algorithm is a metaheuristic optimization technique inspired by natural selection and
 * evolution. It works by:
 * 
 * 1. POPULATION: Start with a set of candidate solutions (individuals). Each individual represents
 *    a complete set of PID gains that could control the robot.
 * 
 * 2. FITNESS: Evaluate how good each individual is. In our case, we run the robot on test paths
 *    and measure the total tracking error (cost function). Lower cost = better fitness.
 * 
 * 3. SELECTION: Pick the best individuals (high fitness) as parents to create the next generation.
 *    We use tournament selection: randomly pick k candidates and keep the best one.
 * 
 * 4. CROSSOVER: Combine two parent genomes to create offspring. We use blend crossover (BLX-alpha):
 *    take the range between two parent values and expand it slightly, then sample a new value.
 *    This creates children that inherit traits from both parents but can exceed both values.
 * 
 * 5. MUTATION: Introduce random changes to offspring genes. We apply Gaussian mutations: add
 *    small random noise to each parameter with a set probability. This prevents convergence to
 *    local minima and explores the search space.
 * 
 * 6. REPEAT: Keep the best individuals (elitism), breed new offspring from the survivors, and
 *    repeat for multiple generations until the best individual stops improving.
 * 
 * ADVANTAGES OF GENETIC ALGORITHMS:
 * - Gradient-free: No need to compute derivatives; works with non-smooth fitness landscapes
 * - Multi-dimensional search: Can optimize 6 PID parameters simultaneously
 * - Parallelizable: Could evaluate multiple individuals concurrently on different paths
 * - Robust: Less likely to get stuck in local minima due to crossover and mutation
 * - Domain-agnostic: No special knowledge about PID control needed
 * 
 * HOW WE USE IT FOR PID TUNING:
 * 
 * GENOME REPRESENTATION:
 * Each individual's genome contains 6 double values:
 *   [transP, transI, transD, rotP, rotI, rotD]
 * 
 * These are the PID gains for:
 *   - Translation (X, Y position errors): P (proportional), I (integral), D (derivative)
 *   - Rotation (heading error): P, I, D
 * 
 * FITNESS FUNCTION:
 * We measure fitness by running each individual on NUM_PATHS test paths (typically 6 different
 * trajectories). For each path:
 *   - Apply the individual's PID gains to the controller
 *   - Execute the path and accumulate error costs from PathPlanner's cost function
 *   - If error exceeds a threshold, abort early and penalize heavily (prevents wild oscillations)
 * 
 * Total fitness = sum of costs across all paths
 * Lower cost = better PID tuning = better fitness
 * 
 * ALGORITHM FLOW:
 * 1. Initialize population with:
 *    - 1 elite seeded with the best known gains from last run
 *    - 2 variants of that elite (jittered slightly)
 *    - 3 random individuals (explore new regions of parameter space)
 * 
 * 2. For each generation:
 *    a) Evaluate all individuals on all test paths
 *    b) Sort by fitness (lower cost is better)
 *    c) Report best individual's gains and cost
 *    d) Select elite individuals to preserve unchanged
 *    e) Fill remainder of population by:
 *       - Tournament selection: pick 2 parents from best individuals
 *       - Crossover: blend their genes to create offspring
 *       - Mutation: randomly perturb offspring genes
 *    f) Repeat with new generation
 * 
 * 3. After GENERATIONS iterations:
 *    - Save best individual's PID gains to persistent preferences
 *    - Next run can seed with these improved values
 * 
 * PARAMETERS TO TUNE:
 * - MUTATION_RATE: Probability each gene mutates (0.30 = 30% chance per gene)
 * - MUTATION_SIGMA: Size of mutations as fraction of parameter range (0.20 = 20% of range)
 * - ELITES: How many top individuals to preserve (3 keeps the 3 best)
 * - GENERATIONS: How many evolution rounds (15 is conservative; more = better but slower)
 * - ALLOWABLE_COST: Abort threshold to prevent bad PID gains from running entire path
 * 
 * PRACTICAL USAGE:
 * 1. Create test paths that represent realistic driving scenarios
 * 2. Enable this command in teleop or a special diagnostic mode
 * 3. Let it run for several generations (5-15 minutes depending on path complexity)
 * 4. Monitor SmartDashboard to watch best cost decrease each generation
 * 5. Once converged (cost stops improving), save the best PID gains
 * 6. Use the improved gains in your main controller
 * 
 * CONVERGENCE INDICATORS:
 * - Best cost is decreasing each generation
 * - Best cost is stabilizing (plateauing)
 * - Population diversity is maintained (not all identical)
 * 
 * When to stop:
 * - After GENERATIONS iterations, algorithm automatically stops
 * - Or manually disable via setRunning(false) if satisfied with current gains
 * 
 * LIMITATIONS & CONSIDERATIONS:
 * - Only as good as your fitness function and test paths
 * - May overfit to test paths and perform poorly on new trajectories
 * - Evaluation is slower than manual tuning (many path runs)
 * - No guarantee of global optimum (may find local optimum)
 * - Environmental factors (battery voltage, traction) change optimal gains
 * 
 * REFERENCES:
 * - Wikipedia: Genetic Algorithm
 * - "Introduction to Genetic Algorithms" - David Goldberg
 * - For robotics: "Evolutionary Robotics" - Nolfi & Floreano
 */

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.FrameConstants;
import frc.robot.subsystems.Locomotion.Drive;
import frc.robot.subsystems.ultilities.FollowPathCommand103;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Random;

public class flowerTest extends Command {
  // -------------------- CONFIGURATION --------------------
  // Number of test paths to evaluate each individual on
  private static final int NUM_PATHS = 6;

  // Genetic Algorithm hyperparameters
  // Population size for each generation
  private static final int POP_SIZE = 6;
  // Number of elite individuals to preserve
  private static final int ELITES = 3;
  // Total generations to evolve
  private static final int GENERATIONS = 15;
  // Probability of mutation per gene
  private static final double MUTATION_RATE = 0.30;
  // Mutation magnitude as fraction of parameter range
  private static final double MUTATION_SIGMA = 0.20;

  // Translational PID bounds (x, y position error)
  private static final double P_MIN_T = 0.0,  P_MAX_T = 10.0;
  private static final double I_MIN_T = 0.0,  I_MAX_T = 0.3;
  private static final double D_MIN_T = 0.0,  D_MAX_T = 1.5;

  // Rotational PID bounds (heading error)
  private static final double P_MIN_R = 0.0,  P_MAX_R = 10.0;
  private static final double I_MIN_R = 0.0,  I_MAX_R = 1;
  private static final double D_MIN_R = 0.0,  D_MAX_R = 1.5;
  
  // Cost threshold to abort path if exceeded
  private static final double ALLOWABLE_COST = 2000.0;
  // Penalty for failed runs
  private static final double HIGH_COST_PENALTY = 1e6;
  // Flag indicating high-cost abort occurred
  private boolean HIGH_COST_FLAG = false;

  // -------------------- STATE --------------------
  // List of PathPlanner paths for evaluation
  private final List<PathPlannerPath> paths = new ArrayList<>();
  // Reference to drive subsystem for path execution
  private final Drive drive;
  // Current path index in evaluation
  private int pathIndex = 0;
  // Currently executing path command
  private FollowPathCommand103 activeCmd;
  // Home/reset command (for cleanup between paths)
  private Command homeCommand;
  // Flag to enable/disable GA tuning
  private boolean running = false;

  // Tunable controller shared across evaluations
  private Optional<TunableHolonomicController> tunableController = Optional.empty();

  // PID parameter container (6 values: trans P/I/D, rot P/I/D)
  private static class Genome {
    // Translational PID gains
    double tP, tI, tD;
    // Rotational PID gains
    double rP, rI, rD;
    
    Genome(double tP, double tI, double tD, double rP, double rI, double rD) {
      this.tP = tP; this.tI = tI; this.tD = tD;
      this.rP = rP; this.rI = rI; this.rD = rD;
    }
    
    // Create independent copy for breeding
    Genome copy() { return new Genome(tP, tI, tD, rP, rI, rD); }
  }

  // Individual candidate with genome and fitness (cost) metric
  private static class Individual {
    // Genetic parameters
    Genome g;
    // Fitness score (lower is better, initialized to infinity)
    double fitness = Double.POSITIVE_INFINITY;
    
    Individual(Genome g) { this.g = g; }
  }

  // Random number generator with fixed seed for reproducibility
  private final Random rng = new Random(2025);

  // Current population of candidates
  private List<Individual> population = new ArrayList<>();
  // Current generation number
  private int generation = 0;
  // Index of individual being evaluated in current generation
  private int indIdx = 0;
  // Accumulated cost across all paths for current individual
  private double accumCostForIndividual = 0.0;

  // Default seed values from last successful run
  private double seedTP = 5.0, seedTI = 0.0, seedTD = 0.0;
  private double seedRP = 5.0, seedRI = 0.0, seedRD = 0.0;

  
  // Initialize flowerTest command with drive subsystem reference
  public flowerTest(Drive drive) {
    this.drive = drive;
    // Load or initialize PID seed values from persistent storage
    loadSeedsFromPreferences();

    // Create tunable controller with seed values
    tunableController = Optional.of(new TunableHolonomicController(
        seedTP, seedTI, seedTD,
        seedTP, seedTI, seedTD,
        seedRP, seedRI, seedRD,
        FrameConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, 
        FrameConstants.kphysicalMaxAngularAccelerationRadiansPerSecond));

    try {
      // Load evaluation paths from PathPlanner files
      paths.add(PathPlannerPath.fromPathFile("W_1"));
      paths.add(PathPlannerPath.fromPathFile("W_2"));
      paths.add(PathPlannerPath.fromPathFile("W_3"));
      paths.add(PathPlannerPath.fromPathFile("W_4"));
      paths.add(PathPlannerPath.fromPathFile("W_5"));
      paths.add(PathPlannerPath.fromPathFile("W_6"));

      // Verify correct number of paths loaded
      if (paths.size() != NUM_PATHS) {
        DriverStation.reportWarning("Expected " + NUM_PATHS + " paths; got " + paths.size(), false);
      }
    } catch (Exception e) {
      DriverStation.reportError("Error loading path: " + e.getMessage(), e.getStackTrace());
    }
  }

  // -------------------- PUBLIC CONTROL --------------------
  // Enable/disable genetic algorithm tuning
  public void setRunning(boolean run) {
    running = run;
  }  // -------------------- COMMAND LIFECYCLE --------------------
  // Initialize GA population and begin first evaluation
  @Override
  public void initialize() {
    // Create initial population with elites and random candidates
    initPopulation();
    generation = 0;
    indIdx = 0;
    pathIndex = 0;
    accumCostForIndividual = 0.0;

    // Apply first individual's genome and start evaluation
    applyGenome(population.get(indIdx).g);
    schedulePath(pathIndex);
  }

  // Execute evaluation cycle: run paths, accumulate costs, advance generation
  @Override
  public void execute() {
    if (!running || activeCmd == null) return;

    // Path still executing - check for cost threshold
    if (!activeCmd.isFinished()) {
      double cost = activeCmd.getCostFunctionAccumulator();
      // Abort if path performance degrades beyond threshold
      if (cost > ALLOWABLE_COST) {
        activeCmd.cancel();
        drive.stopModules();
        HIGH_COST_FLAG = true;
        // Fast-forward to end of individual evaluation
        pathIndex = NUM_PATHS - 1;
        return;
      }
      return; // Continue path execution
    }

    // Path finished - handle end-of-path bookkeeping and progress to next
    if (homeCommand == null || !homeCommand.isScheduled() || homeCommand.isFinished()) {
      double cost = activeCmd.getCostFunctionAccumulator();

      // Accumulate cost for this individual
      accumCostForIndividual += cost;
      if (HIGH_COST_FLAG) {
        accumCostForIndividual += HIGH_COST_PENALTY;
        HIGH_COST_FLAG = false;
      }

      pathIndex++;

      // Schedule next path if available
      if (pathIndex < NUM_PATHS) {
        schedulePath(pathIndex);
        return;
      }

      // Finished evaluating all paths for this individual
      if (pathIndex == NUM_PATHS) {
        population.get(indIdx).fitness = accumCostForIndividual;
        var g = population.get(indIdx).g;

        // Publish best PID values to SmartDashboard
        SmartDashboard.putNumber("PID/transP", g.tP);
        SmartDashboard.putNumber("PID/transI", g.tI);
        SmartDashboard.putNumber("PID/transD", g.tD);
        SmartDashboard.putNumber("PID/rotP", g.rP);
        SmartDashboard.putNumber("PID/rotI", g.rI);
        SmartDashboard.putNumber("PID/rotD", g.rD);
        SmartDashboard.putNumber("PID/cost", population.get(indIdx).fitness);

        System.out.printf("\n[GA] Fitness=%.4f (sum of %d paths)\n", 
            population.get(indIdx).fitness, NUM_PATHS);
        SmartDashboard.putNumber("individualCost", cost);
        SmartDashboard.putNumber("individualIndex", indIdx);

        return;
      }

      // Advance to next individual
      indIdx++;
      if (indIdx < population.size()) {
        pathIndex = 0;
        accumCostForIndividual = 0.0;
        applyGenome(population.get(indIdx).g);
        schedulePath(pathIndex);
      } else {
        // Generation complete - evolve to next generation
        saveBestToPreferences();
        evolvePopulation();
        generation++;
        
        // Check if reached max generations
        if (generation >= GENERATIONS) {
          running = false;
          System.out.printf("[GA] Completed %d generations. Best saved to Preferences.", GENERATIONS);
          return;
        }
        
        // Start evaluating new generation
        indIdx = 0;
        pathIndex = 0;
        accumCostForIndividual = 0.0;
        applyGenome(population.get(indIdx).g);
        schedulePath(pathIndex);
      }
    }
  }

  // Cleanup when command ends
  @Override
  public void end(boolean interrupted) {
    if (interrupted && activeCmd != null && activeCmd.isScheduled()) {
      activeCmd.cancel();
      saveBestToPreferences();
    }
  }

  
  // -------------------- EVALUATION --------------------
  // Schedule path at index for execution
  private void schedulePath(int idx) {
    if (idx < 0 || idx >= paths.size()) {
      DriverStation.reportError("[GA] Bad path index " + idx, false);
      return;
    }
    activeCmd = drive.PathCommandBuilder103(paths.get(idx), tunableController);
    activeCmd.schedule();
  }

  // Push genome's PID gains into controller before path execution
  private void applyGenome(Genome g) {
    tunableController.ifPresent(ctrl -> {
      ctrl.setTranslationGains(g.tP, g.tI, g.tD);
      ctrl.setRotationGains(g.rP, g.rI, g.rD);
    });
  }

  // -------------------- GA CORE --------------------
  // Create initial population: best seed + variations + random
  private void initPopulation() {
    population.clear();
    // Seed elite with current best and jittered variants
    population.add(new Individual(new Genome(seedTP, seedTI, seedTD, seedRP, seedRI, seedRD)));
    population.add(new Individual(jitterAround(seedTP, seedTI, seedTD, seedRP, seedRI, seedRD, 0.15)));
    population.add(new Individual(jitterAround(seedTP, seedTI, seedTD, seedRP, seedRI, seedRD, 0.15)));

    // Fill with random genomes within bounds
    while (population.size() < POP_SIZE) {
      population.add(new Individual(randomGenome()));
    }
  }

  // Create genome with small random perturbations around a base
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

  // Create genome with all parameters randomly generated within bounds
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

  // Evolve population: elitism, tournament selection, crossover, mutation
  private void evolvePopulation() {
    // Sort by fitness (lower cost is better)
    population.sort((a, b) -> Double.compare(a.fitness, b.fitness));

    // Report best individual of generation
    Individual best = population.get(0);
    DriverStation.reportWarning(String.format(
        "[GA] Gen %d best cost=%.5f  tP=%.3f tI=%.3f tD=%.3f  rP=%.3f rI=%.3f rD=%.3f",
        generation, best.fitness,
        best.g.tP, best.g.tI, best.g.tD, best.g.rP, best.g.rI, best.g.rD),
        false);

    // Preserve elite individuals unchanged
    List<Individual> nextPop = new ArrayList<>();
    for (int i = 0; i < ELITES; i++) {
      nextPop.add(new Individual(population.get(i).g.copy()));
    }

    // Fill remainder via tournament selection + crossover + mutation
    while (nextPop.size() < POP_SIZE) {
      // Select two parents via tournament
      Individual p1 = tournamentSelect(3);
      Individual p2 = tournamentSelect(3);
      // Blend parent genomes
      Genome c = blendCrossover(p1.g, p2.g, 0.3);
      // Introduce random mutations
      mutate(c);
      nextPop.add(new Individual(c));
    }

    population = nextPop;
    // Reset fitness for next generation evaluation
    for (var ind : population) ind.fitness = Double.POSITIVE_INFINITY;
  }

  // Tournament selection: pick k random candidates, return best
  private Individual tournamentSelect(int k) {
    Individual best = null;
    for (int i = 0; i < k; i++) {
      Individual cand = population.get(rng.nextInt(population.size()));
      if (best == null || cand.fitness < best.fitness) best = cand;
    }
    return best;
  }

  // BLX-alpha blend crossover: interpolate between two values with overshoot
  private double blend(double a, double b, double alpha, double min, double max) {
    double low = Math.min(a, b);
    double high = Math.max(a, b);
    double range = high - low;
    // Expand range by alpha on both sides, then sample uniformly
    double lo = low - alpha * range;
    double hi = high + alpha * range;
    return clamp(randIn(lo, hi), min, max);
  }

  // Crossover two genomes using blend crossover on each gene
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

  // Apply Gaussian mutation to each gene independently with specified probability
  private void mutate(Genome g) {
    if (rng.nextDouble() < MUTATION_RATE) g.tP = mutateGaussian(g.tP, P_MIN_T, P_MAX_T);
    if (rng.nextDouble() < MUTATION_RATE) g.tI = mutateGaussian(g.tI, I_MIN_T, I_MAX_T);
    if (rng.nextDouble() < MUTATION_RATE) g.tD = mutateGaussian(g.tD, D_MIN_T, D_MAX_T);
    if (rng.nextDouble() < MUTATION_RATE) g.rP = mutateGaussian(g.rP, P_MIN_R, P_MAX_R);
    if (rng.nextDouble() < MUTATION_RATE) g.rI = mutateGaussian(g.rI, I_MIN_R, I_MAX_R);
    if (rng.nextDouble() < MUTATION_RATE) g.rD = mutateGaussian(g.rD, D_MIN_R, D_MAX_R);
  }

  // Add Gaussian-distributed noise to a parameter value
  private double mutateGaussian(double val, double min, double max) {
    double range = max - min;
    double sigma = MUTATION_SIGMA * range;
    double mutated = val + rng.nextGaussian() * sigma;
    return clamp(mutated, min, max);
  }

  // -------------------- PERSIST BEST --------------------
  // Save best individual's genome to persistent preferences
  private void saveBestToPreferences() {
    if (population == null || population.isEmpty()) return;

    var best = population.stream()
        .filter(ind -> !Double.isNaN(ind.fitness))
        .min(Comparator.comparingDouble(ind -> ind.fitness))
        .orElse(population.get(0));
  
    Genome g = best.g;
    Preferences.setDouble("PID/transP", g.tP);
    Preferences.setDouble("PID/transI", g.tI);
    Preferences.setDouble("PID/transD", g.tD);
    Preferences.setDouble("PID/rotP", g.rP);
    Preferences.setDouble("PID/rotI", g.rI);
    Preferences.setDouble("PID/rotD", g.rD);
  }
  
  // Display best individual's PID values on SmartDashboard
  @SuppressWarnings("unused")
  private void DisplayBestPID() {
    if (population == null || population.isEmpty()) return;

    var best = population.stream()
        .filter(ind -> !Double.isNaN(ind.fitness))
        .min(Comparator.comparingDouble(ind -> ind.fitness))
        .orElse(population.get(0));

    Genome g = best.g;

    SmartDashboard.putNumber("PID/transP", g.tP);
    SmartDashboard.putNumber("PID/transI", g.tI);
    SmartDashboard.putNumber("PID/transD", g.tD);
    SmartDashboard.putNumber("PID/rotP", g.rP);
    SmartDashboard.putNumber("PID/rotI", g.rI);
    SmartDashboard.putNumber("PID/rotD", g.rD);
    SmartDashboard.putNumber("PID/cost", best.fitness);
  }

  // -------------------- UTILITIES --------------------
  // Clamp value between min and max bounds
  private double clamp(double v, double min, double max) {
    return Math.max(min, Math.min(max, v));
  }
  
  // Random value uniformly distributed in [min, max]
  private double randIn(double min, double max) {
    return min + rng.nextDouble() * (max - min);
  }
  
  // Random value uniformly distributed in [-magnitude, +magnitude]
  private double randSym(double magnitude) {
    return (rng.nextDouble() * 2 - 1) * magnitude;
  }
  
  // Load PID seed values from persistent preferences or initialize with defaults
  private void loadSeedsFromPreferences() {
    // Initialize preferences if keys don't exist
    if (!Preferences.containsKey("PID/transP")) Preferences.setDouble("PID/transP", seedTP);
    if (!Preferences.containsKey("PID/transI")) Preferences.setDouble("PID/transI", seedTI);
    if (!Preferences.containsKey("PID/transD")) Preferences.setDouble("PID/transD", seedTD);
    if (!Preferences.containsKey("PID/rotP"))   Preferences.setDouble("PID/rotP", seedRP);
    if (!Preferences.containsKey("PID/rotI"))   Preferences.setDouble("PID/rotI", seedRI);
    if (!Preferences.containsKey("PID/rotD"))   Preferences.setDouble("PID/rotD", seedRD);
    
    // Read values and sanitize them
    seedTP = sanitize(Preferences.getDouble("PID/transP", seedTP), 0.0, 18.0);
    seedTI = sanitize(Preferences.getDouble("PID/transI", seedTI), 0.0, 1.0);
    seedTD = sanitize(Preferences.getDouble("PID/transD", seedTD), 0.0, 2.5);
    
    seedRP = sanitize(Preferences.getDouble("PID/rotP", seedRP), 0.0, 18.0);
    seedRI = sanitize(Preferences.getDouble("PID/rotI", seedRI), 0.0, 1.0);
    seedRD = sanitize(Preferences.getDouble("PID/rotD", seedRD), 0.0, 3.0);
  }
  
  // Validate value is not NaN/infinite and within bounds
  private double sanitize(double v, double min, double max) {
    if (Double.isNaN(v) || Double.isInfinite(v)) return min;
    return Math.max(min, Math.min(max, v));
  }
  
  // Reset all PID preferences to default values
  public static void resetPidPreferencesToDefaults() {
    Preferences.setDouble("PID/transP", 5.0);
    Preferences.setDouble("PID/transI", 0.0);
    Preferences.setDouble("PID/transD", 0.0);
    Preferences.setDouble("PID/rotP", 5.0);
    Preferences.setDouble("PID/rotI", 0.0);
    Preferences.setDouble("PID/rotD", 0.0);
  }}
