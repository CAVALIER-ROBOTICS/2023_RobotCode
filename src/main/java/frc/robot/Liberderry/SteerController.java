package frc.robot.Liberderry;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface SteerController {
    MotorController getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();
}