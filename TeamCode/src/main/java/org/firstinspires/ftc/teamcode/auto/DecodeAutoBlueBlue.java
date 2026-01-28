package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Decode Pedro Auto - Blue/Blue", group = "Auton")
public class DecodeAutoBlueBlue extends DecodePedroAutoBase {
    @Override
    protected DecodePedroAutoPlan.Alliance getAlliance() {
        return DecodePedroAutoPlan.Alliance.BLUE;
    }

    @Override
    protected DecodePedroAutoPlan.StartSide getStartSide() {
        return DecodePedroAutoPlan.StartSide.BLUE;
    }
}
