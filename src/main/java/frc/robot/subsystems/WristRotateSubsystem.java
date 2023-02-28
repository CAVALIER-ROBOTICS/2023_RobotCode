// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristRotateSubsystem extends SubsystemBase {
  /** Creates a new WristRotateSubsystem. */
  CANSparkMax wristRotMotor = new CANSparkMax(Constants.wristRotID, MotorType.kBrushless);
  SparkMaxPIDController wristRotPID = wristRotMotor.getPIDController();
  RelativeEncoder wristRotEncoder = wristRotMotor.getEncoder();

  public WristRotateSubsystem() {
    //setting PID
  }

  public void setPercentage(double percent) {
    wristRotMotor.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
