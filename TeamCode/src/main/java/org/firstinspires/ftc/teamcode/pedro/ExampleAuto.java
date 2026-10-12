package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Const;


@Autonomous
public class ExampleAuto extends LinearOpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        Pose start = p.of(24, 24, 90);
        Pose mid = p.of(36, 36, 120);
        Pose end = p.of(48, 48, 180);

        Path line = Paths.line(start, end); // creates a linear path from point a to point b
        Path curve = Paths.curve(start, mid, end); // any middle poses act as control points
        Path path = Paths.path(line, curve); // making a path of paths
        line = line.constant(start); // robot heading is constant
        line = line.tangent(); // robot's heading is tangent to the path
        line = line.linear(start, end); // robot rotates at constant speed if heading changes

        follower.setPose(start);
        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();
        follower.follow(curve);
        while (opModeIsActive() && follower.isBusy()) {
            // .isBusy() returns whether the robot is done with following that path
            // ,update() must be called in every loop of each program to keep on following the path
            follower.update();
        }
        requestOpModeStop();
    }
}