package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.util.Pose2d;

public final class DecodeFieldPositions {
    private DecodeFieldPositions() {
    }

    // TODO: Update these placeholder coordinates with official DECODE manual values.
    public static final Pose2d START_SMALL_TRIANGLE_RED = new Pose2d(0, 0, Math.toRadians(0));
    public static final Pose2d LARGE_TRIANGLE_SET1_RED = new Pose2d(36, 48, Math.toRadians(0));
    public static final Pose2d LARGE_TRIANGLE_SET2_RED = new Pose2d(60, 60, Math.toRadians(0));
    public static final Pose2d SHOOT_POSITION_RED = new Pose2d(24, 0, Math.toRadians(180));
    public static final Pose2d GOAL_RED = new Pose2d(0, 0, Math.toRadians(180));

    public static Pose2d mirrorForBlue(Pose2d redPose) {
        return new Pose2d(redPose.x, -redPose.y, -redPose.heading);
    }
}
