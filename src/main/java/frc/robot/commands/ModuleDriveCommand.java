package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Mk4NeoModule;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class ModuleDriveCommand extends CommandBase {

    private static double DRIVE_DEADBAND = 0.05;
    private static double DRIVE_SPEED_SENSITIVITY = 2.0;

    private final XboxController xbox;
    private final Mk4NeoModule rightFrontModule;
    private final Mk4NeoModule rightRearModule;
    private final Mk4NeoModule leftRearModule;
    private final Mk4NeoModule leftFrontModule;

    private double lastSpeed = 0.0;

    public ModuleDriveCommand(XboxController xbox,
                              Mk4NeoModule rightFrontModule, Mk4NeoModule rightRearModule,
                              Mk4NeoModule leftRearModule, Mk4NeoModule leftFrontModule ) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        //addRequirements();
        this.xbox = xbox;
        this.rightFrontModule = rightFrontModule;
        this.rightRearModule = rightRearModule;
        this.leftRearModule = leftRearModule;
        this.leftFrontModule = leftFrontModule;
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        // get stick values
        double stickY = -xbox.getLeftY();
        double stickX = xbox.getLeftX();

        // preform the deadband adjustment of distance
        double distance = Utl.length(stickY,stickX);
        double speed;
        if (distance < DRIVE_DEADBAND) {
            speed = 0.0;
        } else {
            if (distance > 1.0) {
                distance = 1.0;
            }
            speed = (distance - DRIVE_DEADBAND) / (1.0 - DRIVE_DEADBAND);
        }

        // include the speed sensitivity
        speed = Math.pow(speed, DRIVE_SPEED_SENSITIVITY);
        lastSpeed = speed;

        AngleD direction = new AngleD().atan2(stickX, stickY);
        // set all modules to the same speed
        rightFrontModule.setDirectionAndSpeed(direction, speed);
        rightRearModule.setDirectionAndSpeed(direction, speed);
        leftRearModule.setDirectionAndSpeed(direction, speed);
        leftFrontModule.setDirectionAndSpeed(direction, speed);

    }

    public double getLastSpeed()
    {
        return lastSpeed;
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}
