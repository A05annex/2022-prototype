package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.a05annex.frc.robot.subsystems.ISwerveDrive;
import org.a05annex.util.AngleConstantD;

/**
 * This is a drive subsystem used for drive testing only. It is both a
 * {@link edu.wpi.first.wpilibj2.command.Subsystem}, and implements {@link ISwerveDrive}, so it is a test
 * substitute for our robot serve drive subsystem.
 */
public class DummySwerveSubsystem extends SubsystemBase implements ISwerveDrive {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this TestSwerveDriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DummySwerveSubsystem INSTANCE = new DummySwerveSubsystem();

    /**
     * Returns the Singleton instance of this TestSwerveDriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code TestSwerveDriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DummySwerveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this TestSwerveDriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DummySwerveSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void swerveDriveComponents(double forward, double strafe, double rotation) {

    }

    @Override
    public void prepareForDriveComponents(double forward, double strafe, double rotation) {

    }

    @Override
    public void swerveDrive(AngleConstantD chassisDirection, double speed, double rotation) {

    }

    @Override
    public void swerveDriveFieldRelative(AngleConstantD fieldDirection, double speed, double rotation) {

    }

    @Override
    public void setHeading(AngleConstantD targetHeading) {

    }
}

