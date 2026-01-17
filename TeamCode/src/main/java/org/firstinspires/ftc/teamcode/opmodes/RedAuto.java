package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous
public class RedAuto extends Auto{
    @Override
    protected void setPoses() {
       goalPose = goalPose.mirror();
       startPose = startPose.mirror();
       //pose1 = pose1.mirror();
        //etc
    }


    protected void setColor() {
        Constants.color = Constants.Color.RED;
    }
}
