package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous
public class BlueAuto21 extends Auto{
    @Override
    protected void setPoses() {
        //do nothing
    }

    protected void setColor() {
        Constants.color = Constants.Color.BLUE;
    }

}
