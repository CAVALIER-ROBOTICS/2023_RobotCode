// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PhotonVision;
import frc.robot.subsystems.DriveTrainSubsystems;

public class MatchApriltagCommand extends CommandBase {
  PIDController controller = new PIDController(.09, 0, 0);
  /** Creates a new MatchApriltagCommand. */
  DriveTrainSubsystems dSub;

  public MatchApriltagCommand(DriveTrainSubsystems d) {
    dSub = d;
    addRequirements(dSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget bestTag = PhotonVision.getBest();

    if (bestTag != null) {
      double adjustedYaw = Math.abs(bestTag.getYaw() - dSub.getYaw());
      double set = controller.calculate(adjustedYaw);
      dSub.driveFromNum(0.0, 0.0, set);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
