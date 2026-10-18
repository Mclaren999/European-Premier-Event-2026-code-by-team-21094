package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue Telepopus")
public class BlueTelepopus extends BaseTeleOp {

    @Override protected double getStartX()           { return 15; }
    @Override protected double getStartY()           { return 85; }
    @Override protected double getStartHeadingRad()  { return Math.toRadians(180); }
    @Override protected double getGoalX()            { return 2; }
    @Override protected double getGoalY()            { return 144; }
    @Override protected double getTagFieldX()        { return 16.0; }
    @Override protected double getTagFieldY()        { return 130.0; }
    @Override protected int    getLimelightPipeline(){ return 1; }
    @Override protected boolean isBlue()             { return true; }
}
