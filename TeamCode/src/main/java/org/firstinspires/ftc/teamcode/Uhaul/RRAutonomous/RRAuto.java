package org.firstinspires.ftc.teamcode.Uhaul.RRAutonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.Uhaul.UhaulLinearOpMode;

import kotlin.Unit;
import kotlin.jvm.functions.Function2;

import static org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.DriveConstants.SLOW_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.Uhaul.Tests.RoadRunnerTest.drive.DriveConstants.TRACK_WIDTH;


@Config
@Autonomous(name ="RR Auto Uhaul", group = "rrauto")
public class RRAuto extends UhaulLinearOpMode {

    public MecanumConstraints constraints;
    public MecanumConstraints constraints2;
    public SampleMecanumDriveREVOptimized drive;

    @Override
    public void on_init(){
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        constraints2 = new MecanumConstraints(SLOW_CONSTRAINTS, TRACK_WIDTH);

        robot.drive.initAutonomous();
        robot.latch.initAutonomous();
        robot.lift.initAutonomous();
        robot.grabber.initAutonomous();
        robot.intake.initAutonomous();
        robot.slider.initAutonomous();

        robot.latch.openLatch();

    }

    @Override
    public void run() {


        drive.setPoseEstimate(new Pose2d(-36,-63,Math.toRadians(90)));
        //Starting position is by the middle skystone closest to the skybridge, webcam facing the stones.
        Trajectory trajectory = new TrajectoryBuilder(drive.getPoseEstimate(), new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH))
                //heading is end goal, can be used with tank drive, just drives straight ahead changing angle to get there eventually
                //splineinterpolator changes the angle on the fly (while strafing/driving) to reach the end angle, goes over sometimes to get there
                //linearinterpolator only changes the angle exactly the amount you ask it to
                //constantinterpolator keeps the angle whatever angle you tell it the whole move.
                //setReversed makes the robot drive backwards on x-axis (which is really the y-axis!) to the destination.


                .lineTo(new Vector2d(-30.0, -35.0), new LinearInterpolator(Math.toRadians(90.0), Math.toRadians(90.0)))
                .lineTo(new Vector2d(-30.0, -21.2), new ConstantInterpolator(Math.toRadians(180.0)))
                .build();
        /**start intake*/
        Trajectory trajectory2 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints2)
                .forward(-7.5)
                .build();
        /**stop intake*/
        Trajectory trajectory3 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints)
                .splineTo(new Pose2d(-7.0, -38.0), new LinearInterpolator(Math.toRadians(180.0), Math.toRadians(-180.0)))

                .addMarker(new Vector2d(0.0, -38.0), () -> {
//                 ***Slide out the slider while we are moving!
                    robot.slider.slideOut();

                    return Unit.INSTANCE;
                })

                .lineTo(new Vector2d(40.0,-38.0), new ConstantInterpolator(Math.toRadians(0.0)))
                //
                .lineTo(new Vector2d(50.0, -38.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(-90.0)))
                .build();

        //////////////TURN TO -90 (already included above)

        Trajectory forgotThisOne = new TrajectoryBuilder(drive.getPoseEstimate(), constraints)
                .lineTo(new Vector2d(50.0, -32.5), new ConstantInterpolator(Math.toRadians(-90.0)))
                .build();
        /** lift and place block*/
        Trajectory trajectory4 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints)
                .lineTo(new Vector2d(50.0, -30.0), new ConstantInterpolator(Math.toRadians(-90.0)))
                .build();
        /**latch the latch
        //control award */
        Trajectory trajectory5 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints)
                .lineTo(new Vector2d(33.0, -48.0), new LinearInterpolator(Math.toRadians(-90.0), Math.toRadians(-90.0)))

                .build();
        /**unlatch the latch*/
        Trajectory trajectory6 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints)
                .lineTo(new Vector2d(12.0, -38.0), new ConstantInterpolator(Math.toRadians(-180.0)))
                .lineTo(new Vector2d(-27.0, -38.0), new LinearInterpolator(Math.toRadians(-180.0), Math.toRadians(-90.0)))
                .build();
        /**turn on the intake*/
        Trajectory trajectory7 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints)
                .lineTo(new Vector2d(-27.0, -12.2), new ConstantInterpolator(Math.toRadians(-270.0)))
                .build();
        /**turn off the intake*/
        Trajectory trajectory8 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints)
                .addMarker(0.5, () -> {


                    robot.lift.liftTo(2);

                    return Unit.INSTANCE;
                })
                .addMarker(2.5, () -> {

                    robot.slider.slideOut();

                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(-27.0, -38.0), new LinearInterpolator(Math.toRadians(-270.0), Math.toRadians(90.0)))
                .lineTo(new Vector2d(12.0, -38.0), new ConstantInterpolator(Math.toRadians(-180.0)))
                .splineTo(new Pose2d(33.0, -48.0), new ConstantInterpolator(Math.toRadians(-180.0)))
                .build();
        /**stack the block*/
        Trajectory trajectory9 = new TrajectoryBuilder(drive.getPoseEstimate(), constraints)
                .addMarker(0.5, () -> {

robot.grabber.closeGrabber();
robot.slider.slideIn();
robot.lift.liftTo(1);

                    return Unit.INSTANCE;
                })
                .lineTo(new Vector2d(0.0, -39.0), new ConstantInterpolator(Math.toRadians(-180.0)))
                .build();


        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
        robot.grabber.openGrabber();
        robot.intake.intakeON();
        drive.followTrajectorySync(trajectory2);
        robot.intake.intakeOFF();
        robot.grabber.closeGrabber();
        drive.followTrajectorySync(trajectory3);
        //drive.turnSync(Math.toRadians(-90));
        drive.followTrajectorySync(forgotThisOne);
        robot.grabber.openGrabber();
        drive.followTrajectorySync(trajectory4);
        robot.latch.closeLatch();
        drive.followTrajectorySync(trajectory5);
        robot.latch.openLatch();
        drive.followTrajectorySync(trajectory6);
        robot.intake.intakeON();
        drive.followTrajectorySync(trajectory7);
        robot.intake.intakeOFF();
        robot.grabber.closeGrabber();
        drive.followTrajectorySync(trajectory8); //bring lift up and slider while we are doing this!
        robot.grabber.openGrabber();
        drive.followTrajectorySync(trajectory9); //bring lift downn and slider while we are doing this
      //  drive.setPoseEstimate(new Pose2d(0,0));
      //  drive.setExternalHeading(0);
    }

    public void on_stop(){
        //do something on stop?
    }
}

