package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red Telepopus")
public class RedTelepopus extends BaseTeleOp {

    @Override protected double getStartX()           { return 130; }
    @Override protected double getStartY()           { return 85; }
    @Override protected double getStartHeadingRad()  { return Math.toRadians(0); }
    @Override protected double getGoalX()            { return 145; }
    @Override protected double getGoalY()            { return 146; }
    @Override protected double getTagFieldX()        { return 130.0; }
    @Override protected double getTagFieldY()        { return 130.0; }
    @Override protected int    getLimelightPipeline(){ return 2; }
    @Override protected boolean isBlue()             { return false; }
}
