package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
 class Limelight extends OpMode{
     private Limelight3A limelight;
     private IMU imu;

     @Override
     public void init() {
         limelight = hardwareMap.get(Limelight3A.class,"limelight");
         //limelight.pipelineSwitch(0);//如果limelight内置界面内选择允许Switch须加上这一行
         imu = hardwareMap.get(IMU.class, "imu");
         //RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, );
     }

     @Override
     public void start() {
         limelight.start();

     }

     @Override
     public void loop(){

     }
}
