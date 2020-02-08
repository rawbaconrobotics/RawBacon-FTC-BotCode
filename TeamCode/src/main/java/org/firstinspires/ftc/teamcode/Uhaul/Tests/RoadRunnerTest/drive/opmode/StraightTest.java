package org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name ="Straight Test", group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
