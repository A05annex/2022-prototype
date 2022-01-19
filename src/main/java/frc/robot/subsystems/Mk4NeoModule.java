package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.sensors.CANCoder;
import org.a05annex.util.AngleD;
import org.jetbrains.annotations.NotNull;

import frc.robot.Constants;


import static org.a05annex.util.Utl.*;


/**
 * This class represents and controls an
 * <a href="https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module">SDS MK4</a>
 * Swerve Drive module powered by <a href="https://www.revrobotics.com/rev-21-1650/">REV Neo</a> motors controlled
 * using <a href="https://www.revrobotics.com/rev-11-2158/">REV Spark Max</a> motor controllers and with a
 * <a href="https://store.ctr-electronics.com/cancoder/">CTRE CANcoder</a> as the position encoder for
 * absolute wheel direction.
 */
public class Mk4NeoModule {
    static private final AngleD _PI = new AngleD(AngleD.RADIANS, Math.PI);
    static private final AngleD _NEG_PI = new AngleD(AngleD.RADIANS, -Math.PI);
    static private final AngleD _TWO_PI = new AngleD(AngleD.RADIANS, 2.0 * Math.PI);
    static private final AngleD _PI_OVER_2 = new AngleD(AngleD.RADIANS, Math.PI/2.0);
    static private final AngleD _NEG_PI_OVER_2 = new AngleD(AngleD.RADIANS, -(Math.PI/2.0));

    // This is the physical hardware wired to the roborio
    private final CANSparkMax driveMotor;
    private final CANSparkMax directionMotor;
    private final CANCoder calibrationEncoder;

    // These are the components of the physical hardware
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePID;
    private final RelativeEncoder directionEncoder;
    private final SparkMaxPIDController directionPID;

    // This is the initial 0.0 degree position calibration
    private final double calibrationOffset;
    /**
     * A multiplier for the speed that is either 1.0 (forward) or -1.0 (backwards) because the shortest
     * spin to the desired direction may be the backwards direction of the wheel, which requires the speed
     * to be reversed.
     */
    private double speedMultiplier = 1.0;
    /**
     * The last angle the wheel was set to, in radians. this may be either the front or the back of
     * the wheel - see {@link #speedMultiplier ) documentation for determining whether this is the
     * orientation of the front or the back. This will be in the range -pi to pi.
     */
    private AngleD lastDirection = new AngleD(AngleD.RADIANS, 0.0);
    /**
     * The last direction encoder value that was set. Note, we always set the next spin by using a change angle.
     * This means the encoder setting can be anywhere from -infinity to +infinity.
     */
    private double lastDirectionEncoder = 0.0;
    /**
     * The last speed value that was set for this module, in the range 0.0 to 1.0.
     */
    private double lastSpeed = 0.0;

    private boolean driveBySpeed = true;

    /**
     * * The factory that creates the DriveModule given the
     *
     * @param driveCAN          CAN address for the motor that drives the wheel forward.
     * @param spinCAN           CAN address for the motor that spins the wheel around.
     * @param calibrationCAN    CAN address for the CANcoder.
     * @param calibrationOffset The value of the direction potentiometer that will point the module forward.
     * @return (not null) Returns the initialized drive module.
     */
    public static Mk4NeoModule factory(int driveCAN, int spinCAN, int calibrationCAN, double calibrationOffset) {
        // basic code representations for physical hardware
        CANSparkMax driveMotor = new CANSparkMax(driveCAN, MotorType.kBrushless);
        CANSparkMax spinMotor = new CANSparkMax(spinCAN, MotorType.kBrushless);
        CANCoder calibrationEncoder = new CANCoder(calibrationCAN);
        // derived representations of components embedded in the physical hardware
        RelativeEncoder driveEncoder = driveMotor.getEncoder();
        SparkMaxPIDController drivePID = driveMotor.getPIDController();
        RelativeEncoder spinEncoder = spinMotor.getEncoder();
        SparkMaxPIDController spinPID = spinMotor.getPIDController();
        return new Mk4NeoModule(driveMotor, driveEncoder, drivePID,
                spinMotor, spinEncoder, spinPID,
                calibrationEncoder, calibrationOffset);
    }

    /**
     * Instantiate a DriveModule. All of instanced robot hardware control classes are passed in so this
     * module can be tested using the JUnit test framework.
     *
     * @param driveMotor         (CANSparkMax, not null) The drive motor controller.
     * @param driveEncoder       (RelativeEncoder, not null) The drive motor encoder.
     * @param drivePID           (CANPIDController, not null) The drive motor PID controller.
     * @param directionMotor     (CANSparkMax, not null) The spin motor controller.
     * @param directionEncoder   (RelativeEncoder, not null) The spin motor encoder.
     * @param directionPID       (CANPIDController, not null) The spin motor PID controller.
     * @param calibrationEncoder (CANCoder, not null) The spin analog position encoder which provides
     *                           the absolute spin position of the module.
     * @param calibrationOffset  The value of the analog potentiometer that will point the module forward.
     */
    public Mk4NeoModule(@NotNull CANSparkMax driveMotor, @NotNull RelativeEncoder driveEncoder,
                        @NotNull SparkMaxPIDController drivePID, @NotNull CANSparkMax directionMotor,
                        @NotNull RelativeEncoder directionEncoder, @NotNull SparkMaxPIDController directionPID,
                        @NotNull CANCoder calibrationEncoder, double calibrationOffset) {

        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
        this.drivePID = drivePID;
        this.directionMotor = directionMotor;
        this.directionEncoder = directionEncoder;
        this.directionPID = directionPID;
        this.calibrationEncoder = calibrationEncoder;

        // Initialize the calibration CANcoder
        // TODO: do this ...... see the CANcoder documentation.

        // reset motor controllers to factory default
        this.driveMotor.restoreFactoryDefaults();
        this.directionMotor.restoreFactoryDefaults();

        // invert the spin so positive is a clockwise spin
        this.directionMotor.setInverted(true);

        // update PID controllers for spin and drive motors and initialize them
        initPID(this.drivePID, Constants.DRIVE_kFF, Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_IZONE);
        initPID(this.directionPID, 0.0, Constants.SPIN_kP, Constants.SPIN_kI, 0.0);

        // calibrate
        this.calibrationOffset = calibrationOffset;
        calibrate(); // reset direction encoder position
        this.directionPID.setReference(0.0, CANSparkMax.ControlType.kPosition);
        lastDirection.setValue(AngleD.RADIANS, 0.0);
        lastDirectionEncoder = 0.0;
    }

    /**
     * Updates the spin CANPIDController object using values in constants file. Used only when tuning the PID
     * constants for best control.
     */
    public void setSpinPID() {
        directionPID.setP(Constants.SPIN_kP);
        directionPID.setI(Constants.SPIN_kI);
    }

    /**
     * Updates the drive CANPIDController object using values in constants file. Used only when tuning the PID
     * * constants for best control.
     */
    public void setDrivePID() {
        drivePID.setP(Constants.DRIVE_kP);
        drivePID.setI(Constants.DRIVE_kI);
        drivePID.setFF(Constants.DRIVE_kFF);
        drivePID.setIZone(Constants.DRIVE_IZONE);
    }

    public void setDrivePosPID() {
        drivePID.setP(Constants.DRIVE_POS_kP);
        drivePID.setI(Constants.DRIVE_POS_kI);
        drivePID.setFF(0.0);
        drivePID.setIZone(0.0);
    }

    private void initPID(SparkMaxPIDController pid, double kFF, double kP, double kI, double kIZone) {
        pid.setFF(kFF);
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(0.0);
        pid.setIZone(kIZone);
        pid.setOutputRange(-1.0, 1.0);
    }

    /**
     * Returns the drive motor velocity (RPM) as read from the encoder
     *
     * @return The drive motor velocity (RPM)
     */
    public double getDriveEncoderVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the drive motor position as read from the encoder.
     *
     * @return The drive motor position as read from the encoder.
     */
    public double getDriveEncoderPosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the spin motor position as read from the encoder.
     *
     * @return The spin motor position as read from the encoder.
     */
    public double getSpinEncoderPosition() {
        return directionEncoder.getPosition();
    }

    /**
     * Returns the value of the analog encoder as a double. The value goes from 0.0 to 1.0, wrapping around
     * when the boundary between 0 and 2pi is reached. This method is provided primarily to read the
     * analog encoder to determine the calibrationOffset that should be used for initialization.
     *
     * @return The analog spin encoder position.
     */
    public double getAnalogEncoderPosition() {
        return calibrationEncoder.getAbsolutePosition();
    }

    /**
     * Returns the last speed that was set for this module in m/sec.
     *
     * @return The last speed that was set in m/sec.
     */
    public double getLastSpeed() {
        return lastSpeed * Constants.MAX_DRIVE_VELOCITY;
    }

    /**
     * Returns the last speed that was set for this module normalized to 0.0-1.0.
     *
     * @return the last normalized speed that was set.
     */
    public double getLastNormalizedSpeed() {
        return lastSpeed;
    }

    /**
     * Get the last direction set for the module.
     *
     * @return the last direction set.
     */
    public AngleD getLastDirection() {
        return lastDirection;
    }

    /**
     * Set the NEO direction encoder value using the absolute direction encoder, so that forward is an encoder
     * reading of 0 tics.
     */
    private void calibrate() {
        // (actual - offset) * 360 / 20
        directionEncoder.setPosition((calibrationEncoder.getAbsolutePosition() - calibrationOffset) * 18.0);
    }

//    /**
//     * Set the direction and speed of the drive wheel in this module.
//     *
//     * @param targetDegrees (double) The direction from -180.0 to 180.0 degrees where 0.0 is towards the
//     *                      front of the robot, and positive is clockwise.
//     * @param speed         (double) The normalized speed of the wheel from 0.0 to 1.0 where 1.0 is the maximum
//     *                      forward velocity.
//     */
//    public void setDegreesAndSpeed(double targetDegrees, double speed) {
//        setRadiansAndSpeed(Math.toRadians(targetDegrees), speed);
//    }

    /**
     * Set the module direction in radians. This code finds the closest forward-backward direction and sets the
     * foward-backaward multiplier for speed.
     *
     * @param targetDirection (AngleD) The direction from -pi to pi radians where 0.0 is towards the
     *                      front of the robot, and positive is clockwise.
     */
    public void setDirection(AngleD targetDirection) {
        // The real angle of the front of the wheel is 180 degrees away from the current angle if the wheel
        // is going backwards (i.e. the lastDirection was the last target angle for the module
        AngleD realLastForward = (speedMultiplier > 0.0) ? lastDirection :
                (lastDirection.getRadians() < 0.0) ? lastDirection.add(_PI) : lastDirection.subtract(_PI);
        AngleD deltaRadians = new AngleD(targetDirection).subtract(realLastForward);
        speedMultiplier = 1.0;

        // Since there is wrap-around at -180.0 and 180.0, it is easy to create cases where only a small correction
        // is required, but a very large deltaDegrees results because the spin is in the wrong direction. If the
        // angle is greater than 180 degrees in either direction, the spin is the wrong way. So the next section
        // checks that and changes the direction of the spin is the wrong way.
        if (deltaRadians.getRadians() > _PI.getRadians()) {
            deltaRadians.subtract(_TWO_PI);
        } else if (deltaRadians.getRadians() < _NEG_PI.getRadians()) {
            deltaRadians.add(_TWO_PI);
        }

        // So, the next bit is looking at whether it better to spin the front of the wheel to the
        // target and drive forward, or, to spin the back of the wheel to the target direction and drive
        // backwards - if the spin is greater than 90 degrees (pi/2 radians) either direction, it is better
        // to spin the shorter angle and run backwards.
        if (deltaRadians.getRadians() > _PI_OVER_2.getRadians()) {
            deltaRadians.subtract(_PI);
            speedMultiplier = -1.0;
        } else if (deltaRadians.getRadians() < _NEG_PI_OVER_2.getRadians()) {
            deltaRadians.add(_PI);
            speedMultiplier = -1.0;
        }

        // Compute and set the spin value
        lastDirection = targetDirection;
        lastDirectionEncoder += (deltaRadians.getRadians() * Constants.RADIANS_TO_SPIN_ENCODER);

        directionPID.setReference(lastDirectionEncoder, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Set the direction and speed of the drive wheel in this module.
     *
     * @param targetDirection (double) The direction from -pi to pi radians where 0.0 is towards the
     *                      front of the robot, and positive is clockwise.
     * @param speed         (double) The normalized speed of the wheel from 0.0 to 1.0 where 1.0 is the maximum
     *                      forward velocity.
     */
    public void setDirectionAndSpeed(AngleD targetDirection, double speed) {

        setDirection(targetDirection);

        // Compute and set the speed value
        lastSpeed = speed;
        speed *= Constants.MAX_DRIVE_VELOCITY * speedMultiplier;

        if (!driveBySpeed) {
            setDrivePID();
            driveBySpeed = true;
        }
        drivePID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Set the direction and distance in encoder tics that the module should move. We use this for targeting when
     * the robot is stopped, and we are trying to get very fast response and a very solid lock on the target. This is
     * far more reliable that trying to use a PID to control rotation speed to lock on target.
     *
     * @param targetDirection (AngleD) The direction from -pi to pi radians where 0.0 is towards the
     *                      front of the robot, and positive is clockwise.
     * @param deltaTics     (double) The number of tics the drive motor should mov e.
     */
    public void setDirectionAndDistance(AngleD targetDirection, double deltaTics) {
        setDirection(targetDirection);
        double targetTics = getDriveEncoderPosition() + deltaTics * speedMultiplier;

        if (driveBySpeed) {
            drivePID.setReference(0, CANSparkMax.ControlType.kVelocity);
            setDrivePosPID();
            driveBySpeed = false;
        }
        drivePID.setReference(targetTics, CANSparkMax.ControlType.kPosition);
    }
}
