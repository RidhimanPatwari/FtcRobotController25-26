package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
public class AprilTagLLTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); //config name
        int[] pipelines = {4, 5, 6, 7, 8}; //replace w actual pipeline nums
        int currAT = 20; //disired apriltag num
        limelight.pipelineSwitch(currAT-20); //switch to april tag of num currPL
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot rHOOR = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(rHOOR));
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistFromTag(llResult.getTa());

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

            telemetry.addData("Distance", distance);

            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
            telemetry.addData("Pitch", botPose.getOrientation().getPitch());
            telemetry.addData("Roll", botPose.getOrientation().getRoll());
            telemetry.addData("Botpose", botPose.toString());
        }
    }

    public double getDistFromTag(double ta) {
        //function --> Ax^-2
        //TODO: may or may need need to actually test with our own limlight
        double A = 30665.95;
        double dist = (A/ta);
        return dist;
    }
}