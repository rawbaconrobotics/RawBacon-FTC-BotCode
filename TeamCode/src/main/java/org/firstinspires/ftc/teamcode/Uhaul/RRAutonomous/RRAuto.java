package org.firstinspires.ftc.teamcode.Uhaul.RRAutonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;
import kotlin.jvm.functions.Function2;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name ="RR Auto Uhaul", group = "rrauto")
public class RRAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(5)
                .strafeLeft(6.0)
                .addMarker(() -> {
                    // Do something here

                    return Unit.INSTANCE;
                })
                //heading is end goal, can be used with tank drive, just drives straight ahead changing angle to get there eventually
                //splineinterpolator changes the angle on the fly (while strafing/driving) to reach the end angle, goes over sometimes to get there
                //linearinterpolator only changes the angle exactly the amount you ask it to
                //constantinterpolator keeps the angle whatever angle you tell it the whole move.
                .lineTo(new Vector2d(0.0, -30.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(90.0)))
                .lineTo(new Vector2d(0,2))
                .splineTo(new Pose2d(48.0, -33.0, Math.toRadians(-90.0)), new SplineInterpolator(Math.toRadians(180.0), Math.toRadians(-90.0)))
                .lineTo(new Vector2d(48.0, -26.0), new ConstantInterpolator(Math.toRadians(-90.0)))
                .forward(9)
                .strafeTo(new Vector2d(-20, -20))
                .build();



        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}

