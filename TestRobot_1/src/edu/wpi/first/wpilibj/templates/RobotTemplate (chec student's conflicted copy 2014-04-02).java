/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.DigitalIOButton;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogChannel;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    
    // 8 Motors //
    
    Jaguar driveMotor1;
    Jaguar driveMotor2;
    Jaguar throwing_motor_1;
    Jaguar throwing_motor_2;
    Jaguar arm_motor_right;
    Jaguar arm_motor_left;
    Relay belt_motor_1;
    Relay belt_motor_2;
    
    
    Joystick joy1;
    Timer delayTimer;
//    DigitalInput button;
    DigitalInput armLeftDown_KillSwitch;  //Gathering Arms
    DigitalInput armRightDown_KillSwitch;
    DigitalInput armLeftUp_KillSwitch;
    DigitalInput armRightUp_KillSwitch;
    DigitalInput armThrow_KillSwitch; //Throwing Arm
    Encoder encoder;
    int ticksperrev = 250;
    double pi = 3.1415926535897;
    double wheelDiameter = 4;
    int time ;  
    AnalogChannel sonar;
            
    
    public void robotInit(){
        driveMotor1 = new Jaguar(1);     /* drive motor 1 */
        driveMotor2 = new Jaguar(2);     /* drive motor 2 */
        throwing_motor_1 = new Jaguar(3); //Ball throwing motors
        throwing_motor_2 = new Jaguar(4);
        arm_motor_right = new Jaguar(5); //Moves gather arms up and down
        arm_motor_left = new Jaguar(6);
        belt_motor_1 = new Relay(1); //Gathers ball into bucket
        belt_motor_2 = new Relay(2);
        joy1 = new Joystick(1);
//        button = new DigitalInput(1);
        armRightDown_KillSwitch = new DigitalInput(1);
        armRightUp_KillSwitch = new DigitalInput(2);
        armLeftDown_KillSwitch = new DigitalInput(3);
        armLeftUp_KillSwitch = new DigitalInput(4);
        armThrow_KillSwitch = new DigitalInput(5);
        //encoder = new Encoder(13, 14);
        encoder = new Encoder(13,14);
        encoder.setDistancePerPulse(wheelDiameter*pi/ticksperrev);
        encoder.setReverseDirection(true);
        encoder.start();
        sonar = new AnalogChannel(1);
        
        
    }
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        // throw ball function
        // sense which goal is lit up function
        // move forward function
        driveMotor2.set(.5);
        driveMotor1.set(.5);
        //getDistance();  // get deistance function
        
        //ForwardThenBackSlow();
        //turnRight(90,.1);       // turn right inplace for 90 degrees at 10 percent power
        
        while(isEnabled()){
            System.out.println(armLeftDown_KillSwitch.get());
        }
        
    }
    
    void turnRight(int angle, double turnPower) {
        //code to turn right a given angle
        
        driveMotor1.set(turnPower); // LEft wheel going forward
        driveMotor2.set(turnPower); //right wheel going backward
        double rate = 180*turnPower; // 180 degrees per second at full speed
        Timer.delay(angle/rate);    // angle/rate = time to turn to achieve desired angle
        driveMotor1.set(0);
        driveMotor2.set(0);
    }
    double getDistance() {
        
        double distance = 0;
        // code to calculate distance
            // analogIn1 = sensor1.get()
            //distance = functionX(analongIn1)
    return distance;
    }
    
    
    void ForwardThenBackSlow() {
        double slowForward = .1;
        double slowBackward = -.1;
        double time1 = 5;
        driveMotor1.set(slowForward);
        driveMotor2.set(-slowForward);
        Timer.delay(time1);
        driveMotor1.set(0);
        driveMotor2.set(0);
        Timer.delay(time1);
        driveMotor1.set(slowBackward);
        driveMotor2.set(-slowBackward);
    }
    
    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
       
        //Initiating Variables
        boolean rotateBelt = false;
        boolean armState = false; //initializes the state of the Arm, False is Down, True is Up
        double armAngle =  90; // desired arm rotation angle, degrees
        double twist =1;
        // define the variable that will be used and initiate them to zero
            double speed  = 0 ;
            double speed1 = 0 ;
            double speed2 = 0 ;
        
        while(isEnabled()){
            // get x and y from joystick
            double joyY = joy1.getY();  // joyY is the forward/backward speed
            double joyX =joy1.getX();   // JoyX is the turning speed
                // if robot is turning opposite joystick make joyX = -joyX
            
            // set small joystick values to zero
            if (joyY <=0.05||joyY>=-0.05){
                joyY=0;
            }          
            if (joyX <=0.05||joyX >=-0.05){
                joyX=0;
            }
                                 
            // IF the joystick is displaced in X more that it is dispaced in Y, then the speed of the robot is determined my the X displacement
            if (Math.abs(joyX) > Math.abs(joyY)){ 
                speed = joyX;
            }
            else {
                speed = joyY;
            }  // else speed is determined by the y displacement
            
            // if the robot is not turning
            if (joyX == 0){
                speed1 = speed; // left wheel forwad speed
                speed2 = -speed; // right wheel backward speed
            }
            // turning in place
            else if (joyY == 0){
                speed1 = speed;
                speed2 = speed;
            }
            //turn while moving
            else {
                if (joyX > 0) {   // if turning to the right
                    speed1 = speed; // left motor at speed
                    speed2 = -speed + 1.4*speed*joyY; // at full y right side turns same as left at a slow speed (.4speed)
                                                      // at no y the right side turns oposite the left at full speed
                }
                else {    // if turning to the left
                    speed2 = speed; // right motor at speed
                    speed1 = -speed + 1.4*speed*joyY;  // at full y left side turns same as right at a slow speed (.4speed)
                                                      // at no y the left side turns oposite the right at full speed
                }  
                     
            }
            driveMotor1.set(speed1);
            driveMotor2.set(speed2);
            
        
            
           //Ball gathering mechanism\
            
            if (joy1.getRawButton(5)){ 
                
                if (rotateBelt == false){
                    belt_motor_1.set(Relay.Value.kReverse);
                    belt_motor_2.set(Relay.Value.kForward);
                    rotateBelt = true;
                }
                else {
                    belt_motor_1.set(Relay.Value.kOff);
                    belt_motor_2.set(Relay.Value.kOff); 
                    rotateBelt = false;
                }
            }
            
            double sonarDist = sonar.getAverageVoltage();
            
            
            double lastPrint =0;
            if(Timer.getFPGATimestamp() > lastPrint+2){
//                System.out.println(sonarDist*1000/.997);
                lastPrint = Timer.getFPGATimestamp();
            }
            
            twist = joy1.getTwist();
                        
            if (joy1.getRawButton(4)){ //figure out exactly which button
                armThrow(35, twist); //flings ball arm 'angle' degrees: armThrow(angle)
            }
            
            checkThrowSwitch();

                      
            ///////////////////////////////////////////////
            ///////// ARM CONTROL *WITH* KILL SWITCHES -- all ready to go, just plug them in
            ///////////////////////////////////////////////
            if (joy1.getRawButton(2)){ //figure out exactly which button
                armDown(); //Brings gathering arms down
            }
            else if (joy1.getRawButton(3)){ //figure out exactly which button
                armUp(); //Brings gathering arms up
            }
            else armsOff();

            ///////////////////////////////////////////////
            /////////  THROWING ARM ANGLE SLIDER
            ///////////////////////////////////////////////
            System.out.println(joy1.getTwist()); // Make the twist or gadget between 8 & 9 control power.
            // continuous scan to get value and make it equal to motor power. The value reading is -1 to 1
                 
                        
          }
    }
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
  
    
    void goDistance(double dist, double power){
        System.out.println("We have entered the goDistance method");
        encoder.reset();
        if(power < .2)
            power = .2;
        System.out.println("Going " + dist + " inches with power " + power);
        while(encoder.getDistance()< dist){
            System.out.println(encoder.getDistance());
            driveMotor1.set(power);
        }
        //now we've gone far enough
        driveMotor1.set (0);
    }
    
    
        void armThrow(double angle, double joyPower){ //when you get magnet sensor, replace "time" with "angle"
            System.out.println("We have entered the armThrow method");

            double power = 1*joyPower;  // sets power
            double anglePerTime = 100;// degrees per second
            double reverseArm = .1; //reduction in speed
            double reverseCorrectionFactor = .70; // At half power the ratation speed is different than at full power. this constant corrects for that
            double forwardCorrectionFactor = .92; // Motor 2 runs faster than motor 1
            throwing_motor_1.set(power);
            throwing_motor_2.set(-power*forwardCorrectionFactor);  //motors are facing opposite directions
            Timer.delay(angle/anglePerTime);
            throwing_motor_1.set(-reverseArm);
            throwing_motor_2.set(reverseArm);  //motors are facing opposite directions 
             
             //.4 80 .2 .92 .9 works good for slow movement      
        }    
        
        // Turns off throwing arm if the switch is pressed
        void checkThrowSwitch(){
            if (armThrow_KillSwitch.get() == false){
                throwing_motor_1.set(0);
                throwing_motor_2.set(0);   
            }
        }
        
        void armDown(){ // checks if arm is down, if not keeps going until switch is hit
            System.out.println("We have entered the armDown method");
                                                     
            if (armRightDown_KillSwitch.get()==false){ // the switch reading false means it is pressed
                    arm_motor_right.set(0);
            }
            else arm_motor_right.set(.2);

            if (armLeftDown_KillSwitch.get()==false){ // the switch reading false means it is pressed
                    arm_motor_left.set(0);
            }
             else arm_motor_left.set(-.2);
    }
      
        void armUp(){ //checks if arm is up. if not keeps going until switch is hit
            System.out.println("We have entered the armUp method");  
                
            if (armRightUp_KillSwitch.get()==false){ // the switch reading false means it is pressed
                    arm_motor_right.set(0);
            }
            else arm_motor_right.set(-.2);

            if (armLeftUp_KillSwitch.get()==false){ // the switch reading false means it is pressed
                    arm_motor_left.set(0);
            }
             else arm_motor_left.set(.2); 
        }
        
        void armsOff(){
            arm_motor_right.set(0);
            arm_motor_left.set(0);
        }
        
} //end RobotTemplate
