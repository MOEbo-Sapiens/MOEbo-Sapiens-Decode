package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.States;

@Autonomous
public class BlueAuto extends Auto{
    @Override
    protected void setPoses() {
        //do nothing
    }

    protected void setColor() {
        Constants.color = Constants.Color.BLUE;
    }

}
