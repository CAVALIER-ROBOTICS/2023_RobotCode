package frc.robot.subsystems;

public interface DriveTrainConstants {

  static final double kWheelRadius = 0.0508;
  static final int kEncoderResolution = 4096;

  // USE SYSID FOR THIS INFO
  // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
  public static final double maxVoltage = 7;// 7

  // // this is just an estimate In radians
  // public static final double maxVelocityPerSecond = 7;//2

  // public static final double maxAngularVelocityPerSecond =
  // maxVelocityPerSecond/Math.hypot(0.4041,.4041);

  //HOW TO DO OFFSETS 
  /* when upside down, decreasing offset = turn left/counterclockwise, increasing
     offset = turn right/clockwise
     If it's negative increase */
     
  public static final int pigeonID = 10;

  // public static final MechanicalConfiguration mkGearRatio = new
  // MechanicalConfiguration();

  //FRONT LEFT
  public static final int frontLeftDriveMotor = 1; 
  public static final int frontLeftSteerMotor = 2; 
  public static final int frontLeftSteerEncoder = 52; 
  public static final double frontLeftModuleSteerOffset = Math.toRadians(0); 

  //FRONT RIGHT
  public static final int frontRightDriveMotor = 3;
  public static final int frontRightSteerMotor = 4; 
  public static final int frontRightSteerEncoder = 51; 
  public static final double frontRightModuleSteerOffset = Math.toRadians(0); 

  //BACK LEFT
  public static final int backLeftDriveMotor = 7; 
  public static final int backLeftSteerMotor = 8; 
  public static final int backLeftSteerEncoder = 53;
  public static final double backLeftModuleSteerOffset = Math.toRadians(0); 

  //BACK RIGHT
  public static final int backRightDriveMotor = 5; 
  public static final int backRightSteerMotor = 6;
  public static final int backRightSteerEncoder = 50; 
  public static final double backRightModuleSteerOffset = Math.toRadians(0);

  public static final int volts = 12;
  public static final int voltsSecondsPerMeter = 7;
  public static final int voltSecondsSquaredPerMeter = 5;

  public static final double gearRatio = 5.8 / 1;

}