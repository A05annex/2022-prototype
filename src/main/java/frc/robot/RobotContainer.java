// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ModuleDriveCommand;
import frc.robot.subsystems.Mk4NeoModule;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    public final XboxController xbox = new XboxController(0);

    // subsystem declarations go here unless they are singletons.

    public final Mk4NeoModule rightFrontModule = Mk4NeoModule.factory(
            Constants.CAN_Devices.RF_DRIVE, Constants.CAN_Devices.RF_DIRECTION,
            Constants.CAN_Devices.RF_CALIBRATION, Constants.CalibrationOffset.RF);
    public final Mk4NeoModule rightRearModule = Mk4NeoModule.factory(
            Constants.CAN_Devices.RR_DRIVE, Constants.CAN_Devices.RR_DIRECTION,
            Constants.CAN_Devices.RR_CALIBRATION, Constants.CalibrationOffset.RR);
    public final Mk4NeoModule leftRearModule = Mk4NeoModule.factory(
            Constants.CAN_Devices.LR_DRIVE, Constants.CAN_Devices.LR_DIRECTION,
            Constants.CAN_Devices.LR_CALIBRATION, Constants.CalibrationOffset.LR);
    public final Mk4NeoModule leftFrontModule = Mk4NeoModule.factory(
            Constants.CAN_Devices.LF_DRIVE, Constants.CAN_Devices.LF_DIRECTION,
            Constants.CAN_Devices.LF_CALIBRATION, Constants.CalibrationOffset.LF);

    // Command declarations go here
    public ModuleDriveCommand moduleDriveCommand = new ModuleDriveCommand(
            xbox,  rightFrontModule, rightRearModule, leftRearModule, leftFrontModule);

    private final Command autoCommand = null;
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
    }
    
    public ModuleDriveCommand getModuleDriveCommand()
    {
        return moduleDriveCommand;
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }
}
