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
        
        // Drive forward at half speed
        driveMotor2.set(.5);
        driveMotor1.set(-.5);
        
        //Delete this if you want to implement the distance code below
        Timer.delay(1); ///wait ten seconds
        
        /* Could change all of this a while loop that drives until a certain distance then stops
        double distance = 100; arbitrarily long distance in feet
        double goal = 8; distance from wall in feet
        while(distance > goal){
            distance = 0;
            for(int i=1; i<11; i++){
                distance = getDistance()+ distance;
            }
            distance = distance/10;  // average the 10 measurements.
        }
        */
        
        // Stop
        driveMotor2.set(0);
        driveMotor1.set(0);
        
        armThrow(35, 1); //flings ball arm 'angle' degrees: armThrow(angle, power)
        while(isEnabled()){
            // This method stops the throwing arm on its way down
            checkThrowSwitch();
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
        double distance = sonar.getAverageVoltage()*1000/.997/25.4/12; // distance in feet
        return distance;
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
            
            /**********************************************
            ********** Used for BOTH Drive Methods ********
            ****** First part is used on both methods *******
            **********************************************/
            
            // get x and y from joystick
            double joyY = joy1.getY();  // joyY is the forward/backward speed
            double joyX =-joy1.getX();   // JoyX is the turning speed
                // if robot is turning opposite joystick make joyX = -joyX
           
            // set small joystick values to zero
            if (Math.abs(joyY) <= 0.1){ // If joyY is less than or equal to .05 make it zero
                joyY=0;
            }          
            if (Math.abs(joyX) <= 0.1){ // If joyX is less than or equal to .05 make it zero
                joyX=0;
            }
            
            /**********************************************
            ********** Start Drive Method 1 ******************
            ****** Only use this or Tank Drive Method, *******
            ****** they other must be commented out *******
            **********************************************/
            /*
            // IF the joystick is displaced in X more that it is dispaced in Y, then the speed of the robot is determined my the X displacement
            speed = Math.max(Math.abs(joyX), Math.abs(joyY));
                        
            // if the robot is not turning or if the x value is very small, like it is meant to be zero
            if (Math.abs(joyX) < .1){  
                speed1 = speed; // left wheel forwad speed
                speed2 = -speed; // right wheel backward speed
            }
            // If the robot is turning in place or if y is very small like it is meant to be zero
            else if (Math.abs(joyY) < .1){
                speed1 = speed;
                speed2 = speed;
            }
            //turn while moving ** Both motors move forward
            else {
                if (joyX > 0) {   // if turning to the right
                    speed1 = speed; // left motor at speed
                    speed2 = speed*joyY; // Right motor forward slow, relative to y
                }
                else {    // if turning to the left
                    speed2 = speed; // right motor at speed
                    speed1 = speed*joyY;  // Left motor forward slow at speed
                }  
            }
            driveMotor1.set(speed1);
            driveMotor2.set(speed2);*/
            
            /************ End Drive Method 1 *************/
            
            /**********************************************
            ********** Start Tank Drive Method ******************
            ****** Only use this or Drive Method 1, *******
            ****** they other must be commented out *******
            **********************************************/
            
            
            //double y = -joy1.getY();  // this is not done above for both drive methods
            //double x = joy1.getX();   // this is not done above for both drive methods
            //joyY = -1*joyY; //  inverts Y-axis for tank drive
            
            // Scalar to pinch X and Y axis to lower sensitivity at X,Y= 1,0 and 0,1 in both directions
            joyY = joyY*joyY*joyY; //keeps 1 = 1 and 0=0, but values between 0 and 1 "think" they're closer to 0 than they really are
            joyX = joyX*joyX*joyX; //keeps 1 = 1 and 0=0, but values between 0 and 1 "think" they're closer to 0 than they really are
            /////// turn these ^^ 2 lines off if it's giving you trouble /////////////////
        
            double v = joyY*(2-Math.abs(joyX));
            double w = joyX*(2-Math.abs(joyY));
        
            driveMotor2.set((v-w)/2);  // RIGHT MOTOR  **MAY NEED TO BE MADE NEGATIVE!
            driveMotor1.set(-(v+w)/2);  // LEFT MOTOR
        
            //System.out.println("Left: " + driveMotor1.get() + "      Right: " + driveMotor2.get());
            //System.out.println("X:  " + joyX + "          Y: " + joyY);
            //System.out.println("Left: " + (v+w)/2 + "      Right: " + (v-w)/2);
            
            
            
           /********** End Tank Drive Method ******************/
           
            /********** Start Manual Throwing Arm lowering method ************/
           /*
            if (joy1.getRawButton(7)){
                throwing_motor_1.set(-0.1);
                throwing_motor_2.set(0.1); 
            }
             */     
            
            ////////////////////// MOves one gathering arm only independent of the other //////////
            if (joy1.getRawButton(6)){
                arm_motor_right.set(-.2);
            }
            else{
                arm_motor_right.set(0);
            }
            
             if (joy1.getRawButton(11)){
                arm_motor_right.set(.2);
            }
             else{
                 arm_motor_right.set(0);
            }

            
           /********** Start Ball gathering method ************/
            /////Press button 5 once to start ////////////////
            /////Press button 5 agian to stop ///////////////
            if (joy1.getRawButton(4)){ 
                
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
              ///////////////////////
            //Ungathering ball method//
              ///////////////////////
            if (joy1.getRawButton(5)){ 
                 
                if (rotateBelt == false){
                    belt_motor_1.set(Relay.Value.kForward);
                    belt_motor_2.set(Relay.Value.kReverse);
                    rotateBelt = true;
                }
                else {
                    belt_motor_1.set(Relay.Value.kOff);
                    belt_motor_2.set(Relay.Value.kOff); 
                    rotateBelt = false;
                }
            }
            ////////////////End belt gathering method/////////////////
            
            //**********gET AND PRINT dISTANCE *******//
            //*************Using Sonar*****************//
            double sonarDist = sonar.getAverageVoltage();
            double lastPrint =0;
            if(Timer.getFPGATimestamp() > lastPrint+10){  // increase lastPrint+? to increase  time between print outs
                System.out.println("distance in feet" + sonarDist*1000/.997/25.4/12); // print out distance in feet
                lastPrint = Timer.getFPGATimestamp();
            }
            
            //********This is the value of the twist control at the base of hte joystick ****//
            twist = joy1.getTwist();
            twist = 1-(twist + 1)/2;      
            System.out.println("Twist = " + twist);
            
            //******* This is the method to throw the ball when button 4 is pressed****//                        
            if (joy1.getRawButton(1)){ //figure out exactly which button
                armThrow(35, twist); //flings ball arm 'angle' degrees: armThrow(angle)
                
            }
            
            // This method stops the throwing arm on its way down
            checkThrowSwitch(); 
                      
            ///////////////////////////////////////////////
            ///////// ARM CONTROL *WITH* KILL SWITCHES -- 
            //
            ////Hold button 2 to make the arms go down
            ////Hold button 3 to make the arms go up
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
            System.out.println("Twis" + joy1.getTwist()); // Make the twist or gadget between 8 & 9 control power.
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
/*
            double power = 1*joyPower;  // sets power
            double anglePerTime = 100;// degrees per second
            double reverseArm = .1; //reduction in speed
            double reverseCorrectionFactor = .80; // At half power the ratation speed is different than at full power. this constant corrects for that
            double forwardCorrectionFactor = .92; // Motor 2 runs faster than motor 1
            
            /////// RAISES ARM SLIGHTLY INTO PLACE
            throwing_motor_1.set(0.05*power);
            throwing_motor_2.set(-0.05*power*forwardCorrectionFactor);  //motors are facing opposite directions
            Timer.delay(45/anglePerTime);
            
            /////// ACTUAL THROW
            throwing_motor_1.set(power);
            throwing_motor_2.set(-power*forwardCorrectionFactor);  //motors are facing opposite directions
            Timer.delay((angle-45)/anglePerTime);
            throwing_motor_1.set(-reverseArm);
            throwing_motor_2.set(reverseArm);  //motors are facing opposite directions 
  */           
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
                                                     
            arm_motor_right.set(.2);

            arm_motor_left.set(-.2);
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
