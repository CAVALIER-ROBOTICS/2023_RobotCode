// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.VacuumCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.WristRotateClockCommand;
import frc.robot.commands.WristRotateCounterCommand;
import frc.robot.commands.ArmCommands.ArmAngleCommand;
import frc.robot.commands.ArmCommands.ArmInCommand;
import frc.robot.commands.DriveCommands.FieldDriveCommand;
import frc.robot.commands.DriveCommands.RobotDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.WristRotateSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystems.ArmAngleSubsytem;
import frc.robot.subsystems.ArmSubsystems.ArmExtendSubsystem;
import frc.robot.subsystems.VacuumSubsystems.VacuumSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final XboxController driver = new XboxController(0);
  private static final XboxController operator = new XboxController(1);

  ArmAngleSubsytem armAngleSub = new ArmAngleSubsytem();
  WristSubsystem wristSub = new WristSubsystem();
  WristRotateSubsystem wristRotSub = new WristRotateSubsystem();
  DriveTrainSubsystems driveSub = new DriveTrainSubsystems();

  VacuumSubsystem vacSub = new VacuumSubsystem();
  ArmExtendSubsystem armExtendSubsystem = new ArmExtendSubsystem();

  Trigger armOut = new Trigger(() -> getRightTrigger(operator));
  Trigger armIn = new Trigger(() -> getLeftTrigger(operator));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSub.setDefaultCommand(
        new FieldDriveCommand(
            () -> modifyAxis(driver.getLeftY() *
                DriveTrainSubsystems.maxVelocityPerSecond),
            () -> modifyAxis(driver.getLeftX() *
                DriveTrainSubsystems.maxVelocityPerSecond),
            () -> modifyAxis(driver.getRightX() *
                DriveTrainSubsystems.maxAnglarVelocityPerSecond),
            driveSub));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
        JoystickButton changeDrive = new JoystickButton(driver, 4);

        JoystickButton driveForward = new JoystickButton(driver, 1);
        JoystickButton resetGyro = new JoystickButton(driver, 4);
        JoystickButton setVacuum = new JoystickButton(operator, 1);

        JoystickButton wristClock = new JoystickButton(operator, 7);
        JoystickButton wristCounter = new JoystickButton(operator, 6);
    
        armAngleSub.setDefaultCommand(new ArmAngleCommand(armAngleSub, operator::getLeftY));

        resetGyro.whileTrue(new InstantCommand(driveSub::zeroGyroscope));
        setVacuum.whileTrue(new VacuumCommand(vacSub));
    
        armOut.whileTrue(new ArmInCommand(armExtendSubsystem));
        armIn.whileTrue(new ArmInCommand(armExtendSubsystem));

        wristClock.whileTrue(new WristRotateClockCommand(wristRotSub));
        wristCounter.whileTrue(new WristRotateCounterCommand(wristRotSub));
    
        wristSub.setDefaultCommand(new WristCommand(wristSub, operator::getRightY));

        changeDrive.whileTrue(new InstantCommand(driveSub::zeroGyroscope));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.08);
    SmartDashboard.putNumber("wheelSpeedinput", value);

    // value = Math.copySign(value * value, value);
    return value;
  }

  private static boolean getRightTrigger(XboxController controller) {
    return controller.getRightTriggerAxis() > 0.05;
  }

  public ArmAngleSubsytem getArmAngleSub() {
    return armAngleSub;
  }

  private static boolean getShouldNegate(XboxController controller) {
    return controller.getRightTriggerAxis() < controller.getLeftTriggerAxis();
  }

  private static boolean getLeftTrigger(XboxController controller) {
    return controller.getLeftTriggerAxis() > 0.05;
  }

  private static boolean getOperatorLeftBumper() {
    return driver.getRawButton(3);
  }

  private static boolean getOperatorRightBumper() {
    return driver.getRawButton(1);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.1) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
