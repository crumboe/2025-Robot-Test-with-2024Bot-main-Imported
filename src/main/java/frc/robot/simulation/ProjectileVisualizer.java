package frc.robot.simulation;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ProjectileVisualizer {
    private final DoubleArrayPublisher positionPublisher;
    private final DoubleArrayPublisher velocityPublisher;
    
    public ProjectileVisualizer(String pieceName) {
        NetworkTable table = NetworkTableInstance.getDefault()
            .getTable("AdvantageScope")
            .getSubTable("GamePieces")
            .getSubTable(pieceName);
        
        positionPublisher = table.getDoubleArrayTopic("position").publish();
        velocityPublisher = table.getDoubleArrayTopic("velocity").publish();
    }
    
    public void spawnProjectile(double x, double y, double z, 
                                double vx, double vy, double vz) {
        positionPublisher.set(new double[]{x, y, z});
        velocityPublisher.set(new double[]{vx, vy, vz});
    }
    
    public void close() {
        positionPublisher.close();
        velocityPublisher.close();
    }
}