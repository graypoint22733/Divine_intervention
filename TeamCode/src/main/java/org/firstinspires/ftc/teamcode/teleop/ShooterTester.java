package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Hood;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

@TeleOp
@Config
public class ShooterTester extends OpMode {
    DcMotorEx fM;

    MultipleTelemetry t;

    public static double targetVelocity = 0.0;
    public static double currentVelocity = 0.0;

    public static double hoodPosition = 30.0;

    public static BasicFeedforwardParameters ff = new BasicFeedforwardParameters(1/2400.0, 0.0, 0.05);
    public static PIDCoefficients pids = new PIDCoefficients(0.00003, 0.0, 0.0);

    private final ControlSystem cS = ControlSystem.builder()
            .basicFF(ff)
            .velPid(pids)
            .build();
    Hood hood;

    @Override
    public void init() {
        fM = hardwareMap.get(DcMotorEx.class, "shooter");
        hood = new Hood(hardwareMap);
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        cS.setGoal(new KineticState(0.0, targetVelocity));
        currentVelocity = fM.getVelocity();

        double power = cS.calculate(new KineticState(0.0, currentVelocity));
        fM.setPower(power);
        hood.setAngle(hoodPosition);

        t.addData("power", power);
        t.addData("targetVelocity", targetVelocity);
        t.addData("currentVelocity", currentVelocity);
        t.update();
    }
}
