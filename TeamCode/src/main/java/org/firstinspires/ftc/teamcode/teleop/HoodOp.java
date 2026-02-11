package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


@TeleOp
    public class HoodOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hood hood;

        hood = new Hood(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                hood.setPosition(0);
            }

            telemetry.addData("hoodposition",hood.getAngle());
            telemetry.update();
        }
    }
}
