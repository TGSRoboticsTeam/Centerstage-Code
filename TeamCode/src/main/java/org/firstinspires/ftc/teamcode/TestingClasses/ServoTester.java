package org.firstinspires.ftc.teamcode.TestingClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ServoTester{
   public static double servoUpPos;
   public static double servoDownPos;

   double servoUp;
   double servoDown;

   private Servo servo;

   public ServoTester(HardwareMap hardwareMap, String name, double upPos, double downPos){
      this.servo = hardwareMap.get(Servo.class, name);
      servoUp = upPos;
      servoDown = downPos;
   }

   public void setPositionUp(){
      servo.setPosition(servoUp - servoUpPos);
   }

   public void setPositionDown(){
      servo.setPosition(servoDown - servoDownPos);
   }

   public double getPosition(){
      return servo.getPosition();
   }
}
