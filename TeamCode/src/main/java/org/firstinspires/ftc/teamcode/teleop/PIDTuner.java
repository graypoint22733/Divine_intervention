package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

@Config
@TeleOp(name = "2. PID Tuner", group = "Tuning")
public class PIDTuner extends LinearOpMode {
    //HELP

    // Target angle for all modules to reach
    public static double TARGET_ANGLE = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap);

        telemetry.addData("Status", "Ready to Tune");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Force all modules to the TARGET_ANGLE
            // We trick the drive method: 
            // - Speed (forward/strafe) = 0
            // - Rotation = 0
            // - We manually override the PIDs and Offsets using the Config
            
            // Note: To tune PIDs, we just need the modules to hold a position.
            // We will modify the SwerveDrive to expose a method for this, 
            // OR simpler: we just run the loop and let the internal logic handle it.
            // But since the drive() function calculates vectors based on joystick, 
            // we can't easily force "90 degrees" without joystick math getting in the way.
            
            // INSTEAD: We will update SwerveTeleOpConfig values live, and the 
            // normal drive physics will apply. But to test "Snap to Angle", 
            // we really want to isolate the rotation.
            
            // Let's pass the Config values to the drive system 
            swerve.drive(0, 0, 0, 
                SwerveTeleOpConfig.module1Adjust,
                SwerveTeleOpConfig.module2Adjust,
                SwerveTeleOpConfig.module3Adjust,
                SwerveTeleOpConfig.Kp,
                SwerveTeleOpConfig.Kd,
                SwerveTeleOpConfig.Ki,
                SwerveTeleOpConfig.Kf,
                SwerveTeleOpConfig.Kl,
                false, 1.0, 1.0
            );
            
            // To actually stress-test the PID, we need to artificially inject an error
            // or just physically push the robot module by hand and see if it fights back.
            
            // BETTER APPROACH for this specific request:
            // The user wants to "tell the module to go to a specific angle".
            // Since `SwerveDrive` encapsulates the motor writes, we can't easily override 
            // just one module unless we add a specific `testModule` method to `SwerveDrive`.
            
            // HOWEVER, we can achieve this by hacking the "Offset".
            // If you change the Offset, the module thinks it's in the wrong place and moves.
            
            // REALITY CHECK: To properly tune PID, you want to see a step response.
            // I will add a `tuneModules(targetAngle)` method to SwerveDrive.java below.
            
            swerve.tuneModules(TARGET_ANGLE, 
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