package org.firstinspires.ftc.teamcode.auto;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.util.Pose2d;

public final class DecodePedroAutoPlan {
    public enum Alliance {
        RED,
        BLUE
    }

    public enum StartSide {
        RED,
        BLUE
    }

    public static final class Route {
        public final Pose2d startPose;
        public final Pose2d goalPose;
        public final Pose2d shootPose;
        public final PedroPath intakePathSet1;
        public final PedroPath intakePathSet2;

        private Route(Pose2d startPose, Pose2d goalPose, Pose2d shootPose,
                      PedroPath intakePathSet1, PedroPath intakePathSet2) {
            this.startPose = startPose;
            this.goalPose = goalPose;
            this.shootPose = shootPose;
            this.intakePathSet1 = intakePathSet1;
            this.intakePathSet2 = intakePathSet2;
        }
    }

    private DecodePedroAutoPlan() {
    }

    public static Route build(Alliance alliance, StartSide startSide) {
        Pose2d startPose = startSide == StartSide.RED
                ? DecodeFieldPositions.START_SMALL_TRIANGLE_RED
                : DecodeFieldPositions.mirrorForBlue(DecodeFieldPositions.START_SMALL_TRIANGLE_RED);

        boolean collectionOnRedSide = startSide == StartSide.BLUE;
        Pose2d set1 = collectionOnRedSide
                ? DecodeFieldPositions.LARGE_TRIANGLE_SET1_RED
                : DecodeFieldPositions.mirrorForBlue(DecodeFieldPositions.LARGE_TRIANGLE_SET1_RED);
        Pose2d set2 = collectionOnRedSide
                ? DecodeFieldPositions.LARGE_TRIANGLE_SET2_RED
                : DecodeFieldPositions.mirrorForBlue(DecodeFieldPositions.LARGE_TRIANGLE_SET2_RED);

        Pose2d goalPose = alliance == Alliance.RED
                ? DecodeFieldPositions.GOAL_RED
                : DecodeFieldPositions.mirrorForBlue(DecodeFieldPositions.GOAL_RED);
        Pose2d shootPose = alliance == Alliance.RED
                ? DecodeFieldPositions.SHOOT_POSITION_RED
                : DecodeFieldPositions.mirrorForBlue(DecodeFieldPositions.SHOOT_POSITION_RED);

        PedroPath intakePathSet1 = new PedroPath(Arrays.asList(startPose, set1));
        PedroPath intakePathSet2 = new PedroPath(Arrays.asList(set1, set2));

        return new Route(startPose, goalPose, shootPose, intakePathSet1, intakePathSet2);
    }
}
