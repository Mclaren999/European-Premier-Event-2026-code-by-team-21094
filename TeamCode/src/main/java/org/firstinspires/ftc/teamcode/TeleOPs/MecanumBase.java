package org.firstinspires.ftc.teamcode.TeleOPs;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad.GamepadSubSystem;

@TeleOp(name = "Teleopka")
@Config
public class MecanumBase extends OpMode {

    private Follower follower;
    private FtcDashboard dashboard;
    private PIDController headingPID;
    private GamepadSubSystem DriveOP;

    public static double HEADING_KP = 0.01;
    public static double HEADING_KD = 0.002;
    public static double TURN_SPEED_DEG_PER_SEC = 360;
    public static double HEADING_DEADBAND_DEG = 2;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double headingTargetDeg;
    private boolean xSnapLast = false;

    public void init() {
        DriveOP = new GamepadSubSystem(gamepad1);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        Tuning.follower = follower;

        follower.setPose(new Pose(15, 85, Math.toRadians(180)));
        follower.startTeleOpDrive(true);
        follower.setMaxPower(1);

        headingPID = new PIDController(HEADING_KP, 0, HEADING_KD);
        headingPID.setTolerance(HEADING_DEADBAND_DEG);

        headingTargetDeg = Math.toDegrees(follower.getPose().getHeading());

        loopTimer.reset();
    }

    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        drive(dt);

        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Heading Target", headingTargetDeg);
        telemetry.update();
    }

    private void drive(double dt) {
        DriveOP.readButtons();

        double forward = DriveOP.getLeftY(dt);
        double strafe = DriveOP.getLeftX(dt);
        double turnStick = DriveOP.getRightX(dt);

        double currentHeadingDeg = Math.toDegrees(follower.getPose().getHeading());

        headingTargetDeg += turnStick * TURN_SPEED_DEG_PER_SEC * dt;
        if (gamepad1.x && !xSnapLast) {
            headingTargetDeg += 180;
        }
        xSnapLast = gamepad1.x;

        double headingError = DriveOP.angleErrorDeg(currentHeadingDeg, headingTargetDeg);
        if (abs(headingError) < HEADING_DEADBAND_DEG) headingError = 0;

        double turnPower = -headingPID.calculate(headingError, 0);

        follower.setTeleOpDrive(forward, strafe, turnPower, false);
        follower.update();
    }
}
