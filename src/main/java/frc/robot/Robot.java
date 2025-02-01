// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import javax.lang.model.util.ElementScanner14;
import javax.sound.sampled.Control;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private SparkFlex m_SparkFlex8;
  private SparkClosedLoopController m_ClosedLoopController;
  private double m_TargetPosition = 0; // For absoluteEncoder is [0..1)

  private XboxController m_XboxController;

  private int m_HeartbeatCounter = 0;
  private int m_ResetCounter = 0;
  private int m_SecondCounter = 0;
  private int m_10SecCounter = 0;
  private final int kHearbeatsPerSecond = 50;
  private final double kPositionConversionFactor = 1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_SparkFlex8 = new SparkFlex(8, MotorType.kBrushless);
   
    var m_SparkFlexConfig = new SparkFlexConfig();
    m_SparkFlexConfig.inverted(false);
    m_SparkFlexConfig.idleMode(IdleMode.kBrake); //IdleMode.kCoast);
    m_SparkFlexConfig.absoluteEncoder.positionConversionFactor(kPositionConversionFactor); // is a factor multiplied against absolute encoder's [0..1) when set to 1 will be returned by m_SparkFlex8.getAbsoluteEncoder().getPosition().  Use 360 to get degrees from abs zero.
    m_SparkFlexConfig.absoluteEncoder.velocityConversionFactor(1); 
    m_SparkFlexConfig.absoluteEncoder.zeroOffset(0);
    m_SparkFlexConfig.absoluteEncoder.zeroCentered(false);
    m_SparkFlexConfig.signals.primaryEncoderPositionPeriodMs(5);
    m_SparkFlexConfig.signals.primaryEncoderVelocityPeriodMs(5);
    m_SparkFlexConfig.closedLoop.pidf(0.4,0,0,0);
    m_SparkFlexConfig.closedLoop.velocityFF(1/565);
    m_SparkFlexConfig.closedLoop.outputRange(-1,1);
    m_SparkFlexConfig.closedLoop.maxMotion.maxVelocity(300);  // NeoVortex Max RPM = 6784
    m_SparkFlexConfig.closedLoop.maxMotion.maxAcceleration(100);  // RPM/sec
    m_SparkFlexConfig.closedLoop.maxMotion.allowedClosedLoopError(kPositionConversionFactor*0.005);
    //m_SparkFlexConfig.absoluteEncoder.VoltageCompensationEnabled(true);
    m_SparkFlex8.configure(m_SparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    printMotorAndEncoderConfiguration();

    m_SparkFlex8.set(0);    
    m_XboxController = new XboxController(0);

    m_ClosedLoopController = m_SparkFlex8.getClosedLoopController();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    m_TargetPosition = m_SparkFlex8.getAbsoluteEncoder().getPosition();
    m_ClosedLoopController.setReference(m_TargetPosition, ControlType.kMAXMotionPositionControl);  // in units of rotations
    //m_TargetPosition = kPositionConversionFactor / 4;
    m_HeartbeatCounter = 0;
    //m_ClosedLoopController.setReference(m_TargetPosition, ControlType.kMAXMotionPositionControl);  // in units of rotations
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    
    m_HeartbeatCounter++;

    if (m_HeartbeatCounter % kHearbeatsPerSecond == 0) { 
      m_SecondCounter++;
      if (m_SecondCounter % 10 == 0) {
        m_10SecCounter++;
        m_TargetPosition += kPositionConversionFactor / 4;

        if (m_TargetPosition < 0) m_TargetPosition = kPositionConversionFactor + m_TargetPosition; 
        else if (m_TargetPosition >= kPositionConversionFactor) m_TargetPosition -= kPositionConversionFactor;
        m_ClosedLoopController.setReference(m_TargetPosition, ControlType.kMAXMotionPositionControl);  // in units of rotations
      } 
      System.out.println(
        m_SecondCounter + 
        " Target: " + String.format("%5.3f ",m_TargetPosition) + 
        " Current: " + String.format("%5.3f ",m_SparkFlex8.getAbsoluteEncoder().getPosition()));

      //      m_ClosedLoopController.setReference(m_TargetPosition, ControlType.kPosition);  // in units of rotations

      //m_ClosedLoopController.setReference(m_TargetPosition, ControlType.kMAXMotionPositionControl); //ControlType.kPosition);
    //   m_ClosedLoopController.setReference(m_TargetPosition, ControlType.kPosition);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double speed = deadband(-m_XboxController.getLeftY(),0.06,2);
    if (m_ResetCounter++ % kHearbeatsPerSecond == 0) {
      
      // The absolute encoder is working with SparkFlex controller, NeoVortex motor, 
      // and REV-11-2853 Spark Flex Data Port Breakout Cable.  The motor needs a load too, 
      // for this exampe had a 3:1 gearbox on it. 
      System.out.println(
        "SpeedCmd: " + String.format("%5.3f ",speed) +
        " DriveShaft Absolute Vel: " + String.format("%5.3f ",m_SparkFlex8.getAbsoluteEncoder().getVelocity()) +
        " DriveShaft Absolute Pos: " + String.format("%5.3f ",m_SparkFlex8.getAbsoluteEncoder().getPosition())
        );
    }
    m_SparkFlex8.set(speed);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    m_SparkFlex8.set(0);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void printMotorAndEncoderConfiguration() {
    System.out.println("configAccessor");
    System.out.println("  Inverted: " + m_SparkFlex8.configAccessor.getInverted());
    System.out.println("  IdleMode: " + m_SparkFlex8.configAccessor.getIdleMode());
    System.out.println("  ClosedLoopRampRate: " + String.format("%5.3f ",  m_SparkFlex8.configAccessor.getClosedLoopRampRate()));
    System.out.println("  OpenLoopRampRate: " + String.format("%5.3f ",  m_SparkFlex8.configAccessor.getOpenLoopRampRate()));
    System.out.println("  VoltageCompensation: " + String.format("%5.3f ",  m_SparkFlex8.configAccessor.getVoltageCompensation()));
    System.out.println("  VoltageCompensationEnabled: " + m_SparkFlex8.configAccessor.getVoltageCompensationEnabled());
    System.out.println("configAccessor.AbsoluteEncoder");
    System.out.println("  ZeroOffset: " + String.format("%5.3f ",  m_SparkFlex8.configAccessor.absoluteEncoder.getZeroOffset()));
    System.out.println("  PosConvFactor: " + String.format("%5.3f ",  m_SparkFlex8.configAccessor.absoluteEncoder.getPositionConversionFactor()));
    System.out.println("  VelConvFactor: " + String.format("%5.3f ",  m_SparkFlex8.configAccessor.absoluteEncoder.getVelocityConversionFactor()));
  }

  /** This function provides for zeroing when the value is near zero, and
   *  a rescales so that the truncated deadband then ramps up from zero rather 
   *  than the truncation point (epsilon).  This allows for a very precise and 
   *  continuous funcion from -1 through 1.
   */
  private double deadband(double inValue, double epsilon, double pow){
    double power = (pow <= 0) ? 1 : pow; // fix erroneous "power of" superscript, keep > 0.
    if (Math.abs(inValue) < epsilon ) 
      return 0;  // apply deadband
    else {
      // rescale from epsilon..1 to 0..1
      double sign = inValue < 0 ? -1 : 1;
      final double kMinSpeedAvoidMotorSqueal = 0.03;
      double slope = (1-kMinSpeedAvoidMotorSqueal)/(1-epsilon); // >1
      double reScaleLinearValue = slope * (Math.abs(inValue)-epsilon)+kMinSpeedAvoidMotorSqueal;  // result range 0..1
      // if linear result is too punchy at low end, apply a power > 1.0. Two seems best. To keep linear, set pow to 1.0
      double exponentialSupressionNearZero = sign * Math.pow(reScaleLinearValue,power); // result range 0..1
      return (exponentialSupressionNearZero);
    }
  }
}
