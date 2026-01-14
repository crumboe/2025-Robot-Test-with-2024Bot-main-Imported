package frc.robot.simulation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.ultilities.MovingPlatformShooterSolver.Shooter_Solver;

import java.util.function.Supplier;

public class ShooterSolverSimulation {
    
    private static double robotX;
    private static double robotY;
    private static double robotYaw;
    private static double robotVX ;
    private static double robotVY ;
    private static double targetX ;
    private static double targetY ;
    private static double targetZ ;
    private static double robot_omega;
    private static double shooterYawtoRobot;
    private static Transform2d robotVelocitySup(){
        return new Transform2d(robotVX, robotVY, new Rotation2d(robot_omega));
    }
    private static Pose2d robotPoseSup(){
        return new Pose2d(robotX, robotY, new Rotation2d(robotYaw));
    }
    private static Pose3d targetPoseSup(){
        return new Pose3d(new Translation3d(targetX, targetY, targetZ), new Rotation3d(0,0,0));
    }
    public void main(String[] args) {
        // --- Simulation Parameters ---
        targetX = 12.0; // Target 12 meters away
        targetY = 0.0;
        targetZ = 3.0; // Target height 3 meters
        double shooterMuzzleV0 = 25.0; // Muzzle velocity (m/s)
        shooterYawtoRobot = 0.0; // Shooter angle relative to robot (radians)
        double dt = 0.02; // Time step (20ms)
        double simDuration = 5; // Simulation duration (5 seconds)
        double MAXOMEGA =8.0; // Max robot angular velocity (rad/s)

        // Robot initial state
        robotX = 0.0;
        robotY = 0.0;
        robot_omega = 0.0; // No rotation
        robotYaw = 0.0; // Robot facing forward
        robotVX = 4.0; // Robot velocity in X (m/s)
        robotVY = 2.0; // Robot velocity in Y (m/s)

        // Shooter position relative to the robot
        Translation3d shooterRelativePosition = new Translation3d(0.3, 0.0, 0.5);

        // Target pose supplier
        Supplier<Pose3d> targetPoseSupplier = () -> targetPoseSup();

        // Robot pose supplier
        Supplier<Pose2d> robotPoseSupplier = () -> robotPoseSup();

        // Robot velocity supplier
        Supplier<Transform2d> robotVelocitySupplier = () -> robotVelocitySup();

        // Shooter-to-robot transform supplier
        Supplier<Transform3d> shooterTransformSupplier = () -> new Transform3d(shooterRelativePosition, new Rotation3d(0,0,Math.toRadians(shooterYawtoRobot)));
        Supplier<Double> shooterMuzzleV0Supplier = () -> shooterMuzzleV0;
        // Create the Shooter_Solver instance
        Shooter_Solver solver = new Shooter_Solver(1,
            robotPoseSupplier,
            robotVelocitySupplier,
            shooterTransformSupplier,
            targetPoseSupplier,
            shooterMuzzleV0Supplier
        );
        // Simulation loop
        double time = 0.0;
        System.out.printf("%-10s | %-15s | %-10s | %-15s | %-10s | %-15s | %-10s | %-15s | %-10s\n", 
            "Time (s)", "Robot X/Y (m)", "Yaw (deg)", "Req Yaw (deg)", "Err (deg)", "Omega (rad/s)", "Pitch (deg)", "TOF", "Status");
        while (time <= simDuration) {
            // Solve kinematics
            Shooter_Solver.Solution solution = solver.solveKinematics();

            double omega_command = 0.0;
            double yaw_error_deg = 0.0;    
            // Output results
            String status = solution.status;
            double pitchDeg = Math.toDegrees(solution.requiredShooterPitchRad);
            double yawDeg = Math.toDegrees(solution.requiredRobotYawRad);
            double timeOfFlight = solution.timeOfFlight;

            if (status.equals("Solution Found")) {
                // Simple proportional controller for robot yaw
                yaw_error_deg = yawDeg - robotYaw * 180.0 / Math.PI;
                yaw_error_deg = (yaw_error_deg + 180.0) %360 - 180;
                omega_command = (Math.toRadians(yaw_error_deg) * 16); // P gain of 2.0
                // Clamp to max omega
                if (omega_command > MAXOMEGA) omega_command = MAXOMEGA;
                if (omega_command < -MAXOMEGA) omega_command = -MAXOMEGA;
            } else {
                omega_command = 0.0;
            }

            System.out.printf("%-10.2f | %-7.2f, %-7.2f | %-10.2f | %-15.2f | %-10.2f | %-15.2f | %-10.2f | %-15.2f | %-10s\n",
            time, robotX, robotY, Math.toDegrees(robotYaw), yawDeg, yaw_error_deg, Math.toDegrees(robot_omega), pitchDeg, timeOfFlight, status);
            
            // Simulate ball position over the expected time of flight
            if (status.equals("Solution Found")) {
                double ballTime = 0.0;
                double ballDt = 0.002; // Time step for ball simulation
                // double yawR = yawDeg * Math.PI / 180.0;
                double yawR = robotYaw;
                Pose3d robotPose3d = new Pose3d(new Translation3d(robotPoseSup().getX(),robotPoseSup().getY(), 0.0), 
                                                new Rotation3d(0.0, 0.0, yawR));
                Pose3d shooterPoseWorld =robotPose3d.transformBy(new Transform3d(shooterRelativePosition,new Rotation3d(0,0,shooterYawtoRobot)));
                double ballX = shooterPoseWorld.getX();
                double ballY = shooterPoseWorld.getY();
                double ballZ = shooterPoseWorld.getZ();

                Translation3d ballV0_shooter_centric = new Translation3d(shooterMuzzleV0,0,0)
                                                        .rotateBy(new Rotation3d(0,-solution.requiredShooterPitchRad,0))
                                                        .rotateBy(new Rotation3d(0.0, 0.0, solution.requiredRobotYawRad));
                System.out.print("muzzle speeds sim" + ballV0_shooter_centric.getX() + " " + ballV0_shooter_centric.getY() + " " + ballV0_shooter_centric.getZ() + "\n");
                
                Translation3d shootervelFromRobotRotation = new Translation3d(-robot_omega * shooterTransformSupplier.get().getTranslation().getY(),
                                                        robot_omega * shooterTransformSupplier.get().getTranslation().getX(),
                                                        0);
                shootervelFromRobotRotation = shootervelFromRobotRotation.rotateBy(new Rotation3d(0.0, 0.0, yawR));

                double shooterVelX = robotVX + shootervelFromRobotRotation.getX();
                double shooterVelY = robotVY + shootervelFromRobotRotation.getY();
                double shooterVelZ = 0.0; // Replace with actual Z velocity if available
                Translation3d shooterVelWorld_final = new Translation3d(shooterVelX, shooterVelY, shooterVelZ);
                // System.out.printf("Shooter Velocity simulation (m/s): VX: %.2f, VY: %.2f, VZ: %.2f\n", shooterVelX, shooterVelY, shooterVelZ);
                Translation3d ballV0_world = shooterVelWorld_final.plus(ballV0_shooter_centric);

                // double ballVX = solution.shooterVelocityCheck.getX();
                // double ballVY = solution.shooterVelocityCheck.getY();
                double ballVX = ballV0_world.getX();
                double ballVY = ballV0_world.getY();
                double ballVZ = ballV0_world.getZ();
                System.out.printf("Initial Ball        Velocity (m/s): VX: %.2f, VY: %.2f, VZ: %.2f\n", ballVX, ballVY, ballVZ);
                System.out.printf("Initial Ball Velocity solver (m/s): VX: %.2f, VY: %.2f, VZ: %.2f\n",  solution.shooterVelocityCheck.getX(), 
                                                                                                                solution.shooterVelocityCheck.getY(), 
                                                                                                                solution.shooterVelocityCheck.getZ());
                double gravity = 9.81; // Gravity (m/s^2)

                System.out.println("Ball Trajectory:");
                System.out.printf("%-10s | %-10s | %-10s | %-10s\n", "Time (s)", "X (m)", "Y (m)", "Z (m)");
                while (ballTime <= timeOfFlight) {
                    
                    // Update ball position and velocity
                    ballX += ballVX * ballDt;
                    ballY += ballVY * ballDt;
                    ballZ += ballVZ * ballDt;
                    ballVZ -= gravity * ballDt; // Gravity affects Z velocity
                    
                    ballTime += ballDt;

                    // Print ball trajectory at each step
                }
                System.out.printf("%-10.2f | %-10.2f | %-10.2f | %-10.2f| %-10.2f\n", ballTime, ballX, ballY, ballZ, ballVZ);
            }
            // Update robot state
            robotX += robotVX * dt;
            robotY += robotVY * dt;
            robotYaw += omega_command*dt; // No rotation in this simulation
            // Normalize yaw to the range [-π, π]
            robotYaw = (robotYaw + Math.PI) % (2 * Math.PI) - Math.PI;
            robot_omega = omega_command;
            // Increment time
            time += dt;
        }
    }
}
