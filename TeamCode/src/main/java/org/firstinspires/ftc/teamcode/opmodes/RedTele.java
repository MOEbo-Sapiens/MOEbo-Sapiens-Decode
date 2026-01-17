package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Constants;

@TeleOp
public class RedTele extends Tele{
    @Override
    protected void setPoses() {
        // Red side - mirror the starting pose
        startPose = startPose.mirror();
        goalPose = goalPose.mirror();
        //etc
    }

    protected void setColor() {
        Constants.color = Constants.Color.RED;
    }
}
