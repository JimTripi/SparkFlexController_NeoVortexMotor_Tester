// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

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
  //private SparkFlexConfig m_SparkFlexConfig;
 // private AbsoluteEncoderConfig m_AbsoluteEncoderConfig;
  private XboxController m_XboxController;
  private int m_HeartbeatCounter = 0;
  private final int kUpdateLogHeartbeatInterval = 50;

  public enum algea_shooter {
      INIT,
      INTAKE_ZERO,
      SHOOT,
      UP_90,
      }

  public algea_shooter armPOS;
  
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
    // m_SparkFlexConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_SparkFlex8.configure(m_SparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //RelativeEncoder encoder8 = m_SparkFlex8.getEncoder(); // Not sure why this did not work
    
    m_SparkFlex8.set(0);    
    m_XboxController = new XboxController(0);
    armPOS = algea_shooter.INIT;
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
 
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  
  armPOS = algea_shooter.INIT;
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (armPOS == algea_shooter.INIT) {
      System.out.println("RESET - Going to Zero Position");
      Algea_Reset();
    }
    
    else {
    double speed = deadband(-m_XboxController.getLeftY(),0.06,2);
    
    if(m_XboxController.getYButtonPressed()) {
      m_SparkFlex8.getEncoder().setPosition(0);
      System.out.println("Position Set to Zero");
    }

    if (m_HeartbeatCounter++ % kUpdateLogHeartbeatInterval == 0) {
      System.out.println("Motor Speed:" + String.format("%5.3f",speed));
      System.out.println("Position:"+ String.format("%4.2f", m_SparkFlex8.getEncoder().getPosition())); 
    
    }
    
    if (m_XboxController.getXButtonPressed()) {
      
      m_SparkFlex8.set(0);
      }
      else { m_SparkFlex8.set(speed*0.5);
      }
  }

}
  public void Algea_Reset() {

        // If TRUE Running for First Time
    if ((armPOS == algea_shooter.INIT) && (!m_XboxController.getYButtonPressed())) {
      m_SparkFlex8.set(0.10);
      System.out.println("Entering RESET Code");
    }
    else {
      System.out.println("Hit Sensor for Reset");
      m_SparkFlex8.set(0.0);
      m_SparkFlex8.getEncoder().setPosition(0);
      System.out.println("Init Position:"+ String.format("%4.2f", m_SparkFlex8.getEncoder().getPosition())); 
      armPOS = algea_shooter.INTAKE_ZERO;
    }
    
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
