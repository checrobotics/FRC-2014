/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc1849.BobakCommandRobot.subsystems;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc1849.BobakCommandRobot.Robot;
import org.usfirst.frc1849.BobakCommandRobot.RobotMap;

/**
 *
 * @author bobak
 */
public class DriveSubsystem extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    
    Talon left = RobotMap.driveMotorLeft;
    Talon right = RobotMap.driveMotorRight;
  

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        
    }
    
    public void drive() {
        double y = Robot.oi.joy1.getY();
        double x = Robot.oi.joy1.getX();
        
        double v = (2-(Math.abs(x)*y));
        double w = (2-(Math.abs(y)*x));
        left.set((v+w)/2); 
        right.set((v-w)/2);
        
        System.out.println("Left: " + left.get() + "      Right: " + right.get());
        
    }

}
