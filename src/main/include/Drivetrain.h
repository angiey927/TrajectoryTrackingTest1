// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <wpi/numbers>
#include <AHRS.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() {
    m_gyro.Reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightGroup.SetInverted(true); //8729: no inversion

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.SetDistancePerPulse(2 * wpi::numbers::pi * kWheelRadius /
                                      kEncoderResolution);
    m_rightEncoder.SetDistancePerPulse(2 * wpi::numbers::pi * kWheelRadius /
                                       kEncoderResolution);

    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
  }

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::numbers::pi};  // 1/2 rotation per second

  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t xSpeed,
             units::radians_per_second_t rot);
  void UpdateOdometry();
  void ResetOdometry(const frc::Pose2d& pose);
  frc::Pose2d GetPose() const;

  //CAN Pin Constants
  static const int leftLeadDeviceID = 2;
  static const int leftFollowDeviceID = 3;
  static const int rightLeadDeviceID = 1;
  static const int rightFollowDeviceID = 4;

  //Drive
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

 private:
  static constexpr units::meter_t kTrackWidth = 0.63_m * 2;
  static constexpr double kWheelRadius = 0.0762;  // meters
  static constexpr int kEncoderResolution = 360;

  frc::Encoder m_leftEncoder{0, 1};
  frc::Encoder m_rightEncoder{2, 3};

  frc2::PIDController m_leftPIDController{0.145, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{0.145, 0.0, 0.0};

  //Gyroscope
  AHRS m_gyro{frc::SPI::Port::kMXP};

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  //frc::SimpleMotorFeedforward<units::meters> m_feedforward{1_V, 3_V / 1_mps};
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{1.847_V, 0.1_V / 1_mps, 0.01_V * 1_s * 1_s / 1_m }; //8729: test values on hardwood floor
};
