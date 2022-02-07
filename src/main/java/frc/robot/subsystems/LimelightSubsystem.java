package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.annotation.Target;

public class LimelightSubsystem extends SubsystemBase {
    /**
     * The Singleton instance of this LimelightSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static LimelightSubsystem INSTANCE;

    /**
     * Returns the Singleton instance of this LimelightSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code LimelightSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static LimelightSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LimelightSubsystem();
        }
        return INSTANCE;
    }

    // get limelight NetworkTable
    private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");

    // set pipeline to default from constants initially
    private int m_pipeline = Constants.LIMELIGHT_DEFAULT_PIPELINE;

    // data class
    private static class TargetData {
        double tv;
        double tx;
        double ty;
        double ta;
        double ts;

        TargetData(double tv, double tx, double ty, double ta, double ts) {
            this.tv = tv;
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
            this.ts = ts;
        }
    }

    /**
     * Creates a new instance of this LimelightSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private LimelightSubsystem() {
        // set default pipeline
        setPipeline(m_pipeline);
    }

    public int getPipeline() {
        return m_pipeline;
    }

    public void setPipeline(int pipeline) {
        m_pipeline = pipeline;
        m_table.getEntry("pipeline").setNumber(m_pipeline);
    }

    /**
     * Returns the data class to hold all target data from the limelight.
     * tv (double): Whether the limelight has any valid targets (0 or 1)
     * tx (double): Horizontal offset from crosshair to target (-29.8 degrees to 29.8 degrees)
     * ty (double): Vertical offset from crosshair to target (-24.85 degrees to 24.85 degrees)
     * ta (double): Target area (0% of image to 100% of image)
     * ts (double): Target skew or rotation (-90 degrees to 0 degrees)
     */
    public TargetData getTargetData() {
        return new TargetData(
                m_table.getEntry("tv").getDouble(-1.0),
                m_table.getEntry("tx").getDouble(-1.0),
                m_table.getEntry("ty").getDouble(-1.0),
                m_table.getEntry("ta").getDouble(-1.0),
                m_table.getEntry("ts").getDouble(-1.0)
        );
    }

    /**
     * Prints all target data to SmartDashboard.
     */
    public void printTargetData() {
        TargetData data = getTargetData();
        SmartDashboard.putNumber("tv", data.tv);
        SmartDashboard.putNumber("tx", data.tx);
        SmartDashboard.putNumber("ty", data.ty);
        SmartDashboard.putNumber("ta", data.ta);
        SmartDashboard.putNumber("ts", data.ts);
    }

    @Override
    public void periodic() {
        printTargetData();
    }
}

