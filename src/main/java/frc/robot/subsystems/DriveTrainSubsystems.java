// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrainSubsystems extends SubsystemBase implements DriveTrainConstants {
  /** Creates a new DriveTrainSubsystems. */

  WPI_Pigeon2 pidgey = new WPI_Pigeon2(pigeonID, "OTHERCANIVORE");

  /* Odometry class for tracking robot pose
  Rotation2d.fromDegrees(getFusedHeading()); getRotation2d()
  Rotation2d.fromDegrees(pidgey.getCompassHeading() */

  private final SwerveDriveOdometry odo;

  // These are the modules initialize them in the constructor.
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private int[] moduleDistTraveled = { 0, 0, 0, 0 };

  private double xVelocity;
  private double yVelocity;

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  SwerveModuleState[] states = Constants.m_kinematics.toSwerveModuleStates(chassisSpeeds);
  SwerveModulePosition[] positions = new SwerveModulePosition[4];

  public static final double maxVelocityPerSecond = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  public static final double maxAnglarVelocityPerSecond = maxVelocityPerSecond /
      Math.hypot(Constants.W / 2.0, Constants.L / 2.0);

  public DriveTrainSubsystems() {

    // pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 62200);
    // pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel,
    // 62100);
    // pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro,
    // 62150);

    /* MechanicalConfiguration mechCon = new
    MechanicalConfiguration(DriveTrainConstants.kWheelRadius * 2, 6.12, false,
    12.8, false); */

    MechanicalConfiguration mechCon = SdsModuleConfigurations.MK4_L3;
    MkSwerveModuleBuilder backLeftModuleBuilder = new MkSwerveModuleBuilder();
    MkSwerveModuleBuilder backRightModuleBuilder = new MkSwerveModuleBuilder();
    MkSwerveModuleBuilder frontRightModuleBuilder = new MkSwerveModuleBuilder();
    MkSwerveModuleBuilder frontLeftModuleBuilder = new MkSwerveModuleBuilder();

    backLeftModuleBuilder.withSteerOffset(DriveTrainConstants.backLeftModuleSteerOffset);
    backRightModuleBuilder.withSteerOffset(DriveTrainConstants.backRightModuleSteerOffset);
    frontRightModuleBuilder.withSteerOffset(DriveTrainConstants.frontRightModuleSteerOffset);
    frontLeftModuleBuilder.withSteerOffset(DriveTrainConstants.frontLeftModuleSteerOffset);

    backLeftModuleBuilder.withGearRatio(mechCon);
    backRightModuleBuilder.withGearRatio(mechCon);
    frontRightModuleBuilder.withGearRatio(mechCon);
    frontLeftModuleBuilder.withGearRatio(mechCon);

    backLeftModuleBuilder.withDriveMotor(MotorType.FALCON, DriveTrainConstants.backLeftDriveMotor, "OTHERCANIVORE");
    backLeftModuleBuilder.withSteerMotor(MotorType.FALCON, DriveTrainConstants.backLeftSteerMotor, "OTHERCANIVORE");
    backLeftModuleBuilder.withSteerEncoderPort(DriveTrainConstants.backRightSteerEncoder, "OTHERCANIVORE");
    
    /* backLeftModuleBuilder.withGearRatio(new
    MechanicalConfiguration(backLeftSteerEncoder, backLeftModuleSteerOffset,
    false, backLeftDriveMotor, false)) */

    backRightModuleBuilder.withDriveMotor(MotorType.FALCON, DriveTrainConstants.backRightDriveMotor, "OTHERCANIVORE");
    backRightModuleBuilder.withSteerMotor(MotorType.FALCON, DriveTrainConstants.backRightSteerMotor, "OTHERCANIVORE");
    backRightModuleBuilder.withSteerEncoderPort(DriveTrainConstants.backLeftSteerEncoder, "OTHERCANIVORE");

    frontRightModuleBuilder.withDriveMotor(MotorType.FALCON, DriveTrainConstants.frontRightDriveMotor, "OTHERCANIVORE");
    frontRightModuleBuilder.withSteerMotor(MotorType.FALCON, DriveTrainConstants.frontRightSteerMotor, "OTHERCANIVORE");
    frontRightModuleBuilder.withSteerEncoderPort(DriveTrainConstants.frontRightSteerEncoder, "OTHERCANIVORE");

    frontLeftModuleBuilder.withDriveMotor(MotorType.FALCON, DriveTrainConstants.frontLeftDriveMotor, "OTHERCANIVORE");
    frontLeftModuleBuilder.withSteerMotor(MotorType.FALCON, DriveTrainConstants.frontLeftSteerMotor, "OTHERCANIVORE");
    frontLeftModuleBuilder.withSteerEncoderPort(DriveTrainConstants.frontLeftSteerEncoder, "OTHERCANIVORE");

    frontLeftModule = frontLeftModuleBuilder.build();
    frontRightModule = frontRightModuleBuilder.build();
    backRightModule = backRightModuleBuilder.build();
    backLeftModule = backLeftModuleBuilder.build();

    /* CANCoder fl = (CANCoder) frontLeftModule.getSteerEncoder();
    fl.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition,
    10); */

    updatePositions();
    odo = new SwerveDriveOdometry(Constants.m_kinematics, pidgey.getRotation2d(), positions);

  }

  public SwerveModulePosition getPos(SwerveModule state) {
    /* SwerveDrivePoseEstimator estimator = new
    SwerveDrivePoseEstimator(Constants.m_kinematics, getGyroscopeRotation(),
    positions, getPose()); */

    return state.getPosition();
  }

  public void zeroGyroscope() {
    pidgey.reset();
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(360.0 - pidgey.getAngle());
  }

  public void drive(ChassisSpeeds speeds) {
    states = Constants.m_kinematics.toSwerveModuleStates(speeds);

    /* states = Constants.m_kinematics.toSwerveModuleStates(speeds);
    frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocityPerSecond *
    maxVoltage, states[0].angle.getRadians()); */

    frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage,
        states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage,
        states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage,
        states[2].angle.getRadians());
    backRightModule.set(-states[3].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage,
        states[3].angle.getRadians());
  }

  public void steer(ChassisSpeeds speeds, double controllerRot) {
    states = Constants.m_kinematics.toSwerveModuleStates(speeds);

    if (Math.abs(controllerRot) > 0) {
      SmartDashboard.putBoolean("Steerin", true);
      frontLeftModule.set(states[0].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage,
          states[0].angle.getRadians());
      frontRightModule.set(states[1].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage,
          states[1].angle.getRadians());
      backLeftModule.set((states[2].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage),
          -states[2].angle.getRadians());
      backRightModule.set(states[3].speedMetersPerSecond / maxVelocityPerSecond * maxVoltage,
          -states[3].angle.getRadians());
    } else {
      SmartDashboard.putBoolean("Steerin", true);
      drive(speeds);
    }
  }

  public void setModules(SwerveModuleState[] speeds) {
    states = speeds;
    frontLeftModule.set(speeds[0].speedMetersPerSecond / -Constants.AutoConstants.maxSpeedMetersPerSecond * maxVoltage,
        speeds[0].angle.getRadians());
    frontRightModule.set(speeds[1].speedMetersPerSecond / -Constants.AutoConstants.maxSpeedMetersPerSecond * maxVoltage,
        speeds[1].angle.getRadians());
    backLeftModule.set(speeds[2].speedMetersPerSecond / -Constants.AutoConstants.maxSpeedMetersPerSecond * maxVoltage,
        speeds[2].angle.getRadians());
    backRightModule.set(speeds[3].speedMetersPerSecond / -Constants.AutoConstants.maxSpeedMetersPerSecond * maxVoltage,
        speeds[3].angle.getRadians());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("BackRight", Math.toDegrees(backRightModule.getSteerAngle()));
    SmartDashboard.putNumber("BackLeft", Math.toDegrees(backLeftModule.getSteerAngle()));
    SmartDashboard.putNumber("FrontRight", Math.toDegrees(frontRightModule.getSteerAngle()));
    SmartDashboard.putNumber("FrontLeft", Math.toDegrees(frontLeftModule.getSteerAngle()));
  }

  public SwerveModuleState[] invert(SwerveModuleState[] x) {
    SwerveModuleState[] temp = new SwerveModuleState[4];
    for (int i = 3; i >= 0; i--) {
      temp[i] = new SwerveModuleState((x[i].speedMetersPerSecond), makeRotNegative(x[i].angle));
    }
    return temp;
  }

  Rotation2d makeRotNegative(Rotation2d r2) {
    return new Rotation2d(-r2.getRadians());
  }

  public SwerveModuleState[] makeNegative(SwerveModuleState[] x) {
    SwerveModuleState[] temp = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      temp[i] = new SwerveModuleState((-x[i].speedMetersPerSecond), makeRotNegative(x[i].angle));
    }
    return temp;
  }

  public void updatePositions() {
    positions[0] = frontLeftModule.getPosition();
    positions[1] = frontRightModule.getPosition();
    positions[2] = backLeftModule.getPosition();
    positions[3] = backRightModule.getPosition();

    // positions[0] = new SwerveModulePosition();
    // positions[1] = new SwerveModulePosition();
    // positions[2] = new SwerveModulePosition();
    // positions[3] = new SwerveModulePosition();

    SmartDashboard.putNumber("FrontLeftPos", positions[0].distanceMeters);
  }

  public void updateOdo() {
    updatePositions();
    SwerveModuleState[] temp = invert(states);
    odo.update(pidgey.getRotation2d(), positions);

    // SmartDashboard.putString("Odo", ""+odo.getPoseMeters());
  }

  // private double unitsToDistance(double sensorCounts){
  // double motorRotations = (double)sensorCounts / 2048;
  // double wheelRotations = motorRotations / 6.75;
  // double positionMeters = wheelRotations * (2 * Math.PI *
  // Units.inchesToMeters(2));
  // return positionMeters;
  // }

  public Pose2d getPose() {
    return odo.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odo.resetPosition(pose.getRotation(), positions, pose);
  }

  public void fieldOrientedDrive(DoubleSupplier xtrans, DoubleSupplier ytrans, DoubleSupplier rot) {
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xtrans.getAsDouble(),
            ytrans.getAsDouble(),
            rot.getAsDouble(),
            getGyroscopeRotation()));
    xVelocity = xtrans.getAsDouble();
    yVelocity = ytrans.getAsDouble();
  }

  public void robotOrientedDrive(double xtrans, double ytrans, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(
        xtrans,
        ytrans,
        rot);

    drive(speeds);
  }

  public void driveFromNum(Double xtrans, Double ytrans, Double rot) {
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xtrans,
            ytrans,
            rot,
            getGyroscopeRotation()));
    xVelocity = xtrans;
    yVelocity = ytrans;
  }

  public void driveFromNumRobotOriented(Double xtrans, Double ytrans, Double rot) {
    drive(
        new ChassisSpeeds(
            xtrans,
            ytrans,
            rot));
    xVelocity = xtrans;
    yVelocity = ytrans;
  }

  public void stop() {
    driveFromNum(0.0, 0.0, 0.0);
  }

  public double getSpeed() {
    double speed = Math.sqrt(Math.pow(2, xVelocity) + Math.pow(2, yVelocity));
    return speed;
  }

  public double getRoll() {
    double roll = pidgey.getRoll();
    SmartDashboard.putNumber("Roll", roll);
    return roll;
  }

  public double getPitch() {
    double pitch = pidgey.getPitch();
    SmartDashboard.putNumber("Pitch", pitch);
    return pitch;
  }

  public double getYaw() {
    return pidgey.getYaw();
  }

  public SwerveModuleState[] getModules() {
    return states;
  }
}
