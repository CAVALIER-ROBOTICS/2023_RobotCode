// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristRotateSubsystem;

public class WristRotateClockCommand extends CommandBase {
  /** Creates a new WristRotCommand. */
  WristRotateSubsystem wristRotSub;
  public WristRotateClockCommand(WristRotateSubsystem wSub) {
    wristRotSub = wSub;
    addRequirements(wristRotSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristRotSub.setPercentage(.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristRotSub.setPercentage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
