// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

// THIS IS THE CONTROLLER IMPORT (It is important and took us a while to find)
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Timer;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;


 /* This is a demo program showing the use of the RobotDrive class, specifically it contains the code
  necessary to operate a robot with tank drive. */

public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private JoystickButton Lbutton1;
  private JoystickButton Lbutton2;
  /*private JoystickButton Lbutton3;
  private JoystickButton Lbutton4;
  private JoystickButton Lbutton5;
  private JoystickButton Lbutton6;
  private JoystickButton Lbutton7;
  private JoystickButton Lbutton8;*/
  

  private VictorSP m_left = new VictorSP(6);
  private VictorSP m_right = new VictorSP(2);

  private final Timer m_timer = new Timer();

  private DoubleSolenoid Arm;



  @Override
  public void robotInit() {
    //DifferentialDrive is another word for tank drive and is used on the 2019 robot
    m_myRobot = new DifferentialDrive(m_left, m_right);
    
    /* Joysticks we use are numbered and need to be coordinated need to be defined as the left or right
    joystick to ensure that we are not driving the wrong way*/
    m_leftStick = new Joystick(1);
    m_rightStick = new Joystick(0);


     //These are the left joystick buttons
      Lbutton1 = new JoystickButton(m_leftStick, 1);
      Lbutton2 = new JoystickButton(m_leftStick, 2);
      /*Lbutton3 = new JoystickButton(m_leftStick, 3);
      Lbutton4 = new JoystickButton(m_leftStick, 4);
      Lbutton5 = new JoystickButton(m_leftStick, 5);
      Lbutton6 = new JoystickButton(m_leftStick, 6);
      Lbutton7 = new JoystickButton(m_leftStick, 7);
      Lbutton8 = new JoystickButton(m_leftStick, 8);*/
    
    /*This tells the robot that all inputs are inverted which tells makes our robot drive forwards
    and not backwards*/
    m_left.setInverted(true);
    m_right.setInverted(true);

    Arm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }

/* This function is run once each time the robot enters autonomous mode. This function is also imporant
because it resets the timer to 0 and then starts the timer at the begining of the autonomous period
so our robot knows to only drive for 2 seconds */
@Override
public void autonomousInit() {
  m_timer.reset();
  m_timer.start();
}

  /** This function is called periodically during autonomous. */
@Override
public void autonomousPeriodic() {
  // Drive for 2 seconds
  if (m_timer.get() < 2.0) {
    m_myRobot.tankDrive(-0.5, -0.5); // drives backwards at half speed (We need to figure out how to invert)
  } 
  else if (m_timer.get() < 3.25)
  {
    m_myRobot.tankDrive(0.7, -0.7);
  }
  else if(m_timer.get() < 5)
  {
    m_myRobot.tankDrive(-0.5, -0.5);
  }
  else {
    m_myRobot.stopMotor(); // stops the robot
  }  
}

  //This is the teleoperated mode that you select from the RoboRio control panel (I forgot what is was called)
  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
    
    Boolean ArmButtonPressed = m_leftStick.getRawButton(1);
    if (ArmButtonPressed) {
      Arm.set(kForward);
    }

    Boolean ArmButtonReleased = m_leftStick.getRawButton(2);
    if (ArmButtonReleased) {
      Arm.set(kReverse);
    }

    //Lbutton1.whenPressed(() -> Arm.set(kForward));
    
    //Lbutton2.whenPressed(() -> Arm.set(kReverse));

  }
}