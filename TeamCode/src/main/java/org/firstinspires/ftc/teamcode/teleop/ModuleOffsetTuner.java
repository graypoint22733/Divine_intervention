package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

@TeleOp(name = "3. Module Offset Tuner", group = "Tuning")
public class ModuleOffsetTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);

        telemetry.addData("Status", "Holding Zero.");
        telemetry.addData("Instructions", "Adjust 'moduleAdjust' in Dashboard until wheels are straight.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // We reuse the tuneModules method from the PID tuner!
            // We tell the robot "Go to Angle 0".
            // If the offset is wrong, "Angle 0" will look crooked.
            // By changing the offset, you fix "Angle 0".
            
            swerve.tuneModules(0, // Target Angle = 0 (Forward)
                SwerveTeleOpConfig.module1Adjust,
                SwerveTeleOpConfig.module2Adjust,
                SwerveTeleOpConfig.module3Adjust,
                SwerveTeleOpConfig.Kp,
                SwerveTeleOpConfig.Kd,
                SwerveTeleOpConfig.Ki,
                SwerveTeleOpConfig.Kf,
                SwerveTeleOpConfig.Kl
            );

            telemetry.update();
        }
    }
}