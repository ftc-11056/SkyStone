package org.firstinspires.ftc.teamcode.PP;


public class Paths_Library_Red_3Cubes {


    private static OurPoint[] Left1Points = {
            new OurPoint(1.566, -0.8325, 143),
            new OurPoint(0.8, -1.37, 143),
            new OurPoint(0.673, -1.52, 180),
            new OurPoint(0.55, -1.63, 180)};
    private static double toleranceLeft1 = 75;
    private static double KcLeft1 = 1.5;
    private static double MaxVelocityLeft1 = 0.6;
    private static double turnSpeedLeft1 = 0.4;
    private static boolean frontLeft1 = true;
    private static Path Left1 = new Path(Left1Points, toleranceLeft1, KcLeft1, MaxVelocityLeft1, turnSpeedLeft1, frontLeft1);

    private static OurPoint[] LeftFoundationPoints = {
            new OurPoint(0.55, -1.63, 180),
            new OurPoint(0.92, -1.3, 180),
            new OurPoint(1, 0.8, 180),
            new OurPoint(1.2, 1, 270),
            new OurPoint(0.95, 1.37, 270),
            new OurPoint(0.75, 1.25, 270)};
    private static double toleranceLeftFoundation = 85;
    private static double KcLeftFoundation = 1.5;
    private static double MaxVelocityLeftFoundation = 1.5;
    private static double turnSpeedLeftFoundation = 0.4;
    private static boolean frontLeftFoundation = false;
    private static Path LeftFoundation = new Path(LeftFoundationPoints, toleranceLeftFoundation, KcLeftFoundation, MaxVelocityLeftFoundation, turnSpeedLeftFoundation, frontLeftFoundation);

    private static OurPoint[] FoundationBackPoints = {
            new OurPoint(0.75, 1.25, 270),
            new OurPoint(1.35, 1.25, 270)};
    private static double toleranceFoundationBack = 120;
    private static double KcFoundationBack = 10;
    private static double MaxVelocityFoundationBack = 1;
    private static double turnSpeedFoundationBack = 0.3;
    private static boolean frontFoundationBack = true;
    private static Path FoundationBack = new Path(FoundationBackPoints, toleranceFoundationBack, KcFoundationBack, MaxVelocityFoundationBack, turnSpeedFoundationBack, frontFoundationBack);

    private static OurPoint[] Left2Points = {
            new OurPoint(1.35, 1.25, 180),
            new OurPoint(1.1, 0.6, 180),
            new OurPoint(1, -0.3, 140),
            new OurPoint(1, -0.65, 140),
            new OurPoint(0.67, -0.8, 140),
            new OurPoint(0.35, -1.25, 140)};
    private static double toleranceLeft2 = 120;
    private static double KcLeft2 = 10;
    private static double MaxVelocityLeft2 = 1;
    private static double turnSpeedLeft2 = 1;
    private static boolean frontLeft2 = true;
    private static Path Left2 = new Path(Left2Points, toleranceLeft2, KcLeft2, MaxVelocityLeft2, turnSpeedLeft2, frontLeft2);

    private static OurPoint[] LeftFoundation2Points = {
            new OurPoint(0.35, -1.25, 143),
            new OurPoint(1, -0.7, 180),
            new OurPoint(1.1, 0.5, 173),
            new OurPoint(1.25, 1.1, 173)};
    private static double toleranceLeftFoundation2 = 85;
    private static double KcLeftFoundation2 = 2.5;
    private static double MaxVelocityLeftFoundation2 = 1.5;
    private static double turnSpeedLeftFoundation2 = 0.5;
    private static boolean frontLeftFoundation2 = false;
    private static Path LeftFoundation2 = new Path(LeftFoundation2Points, toleranceLeftFoundation2, KcLeftFoundation2, MaxVelocityLeftFoundation2, turnSpeedLeftFoundation2, frontLeftFoundation2);

    private static OurPoint[] Left3Points = {
            new OurPoint(1.25, 1.1, 180),
            new OurPoint(1.1, 0.6, 180),
            new OurPoint(1, -0.1, 110),
            new OurPoint(0.87, -0.35, 110),
            new OurPoint(0.25, -0.65, 110)};
    private static double toleranceLeft3 = 95;
    private static double KcLeft3 = 2.5;
    private static double MaxVelocityLeft3 = 1.5;
    private static double turnSpeedLeft3 = 0.5;
    private static boolean frontLeft3 = true;
    private static Path Left3 = new Path(Left3Points, toleranceLeft3, KcLeft3, MaxVelocityLeft3, turnSpeedLeft3, frontLeft3);

    private static OurPoint[] LeftFoundation3Points = {
            new OurPoint(0.25, -0.7, 140),
            new OurPoint(0.95, -0.45, 180),
            new OurPoint(1, 0.5, 150),
            new OurPoint(1.6, 1, 150)};
    private static double toleranceLeftFoundation3 = 100;
    private static double KcLeftFoundation3 = 2.5;
    private static double MaxVelocityLeftFoundation3 = 1.5;
    private static double turnSpeedLeftFoundation3 = 0.6;
    private static boolean frontLeftFoundation3 = false;
    private static Path LeftFoundation3 = new Path(LeftFoundation3Points, toleranceLeftFoundation3, KcLeftFoundation3, MaxVelocityLeftFoundation3, turnSpeedLeftFoundation3, frontLeftFoundation3);


    //    TODO Center Points
    private static OurPoint[] Center1Points = {
            new OurPoint(1.566, -0.8325, 143),
            new OurPoint(1.2, -1, 140),
            new OurPoint(0.48, -1.52, 140)};
    private static double toleranceCenter1 = 75;
    private static double KcCenter1 = 0.5;
    private static double MaxVelocityCenter1 = 0.6;
    private static double turnSpeedCenter1 = 0.4;
    private static boolean frontCenter1 = true;
    private static Path Center1 = new Path(Center1Points, toleranceCenter1, KcCenter1, MaxVelocityCenter1, turnSpeedCenter1, frontCenter1);

    private static OurPoint[] CenterFoundationPoints = {
            new OurPoint(0.48, -1.52, 180),
            new OurPoint(0.95, -0.7, 180),
            new OurPoint(0.95, 0.55, 270),
            new OurPoint(0.5, 1.20, 270)};
    private static double toleranceCenterFoundation = 85;
    private static double KcCenterFoundation = 1.5;
    private static double MaxVelocityCenterFoundation = 1.5;
    private static double turnSpeedCenterFoundation = 1.1;
    private static boolean frontCenterFoundation = false;
    private static Path CenterFoundation = new Path(CenterFoundationPoints, toleranceCenterFoundation, KcCenterFoundation, MaxVelocityCenterFoundation, turnSpeedCenterFoundation, frontCenterFoundation);

    private static OurPoint[] Center2Points = {
            new OurPoint(0.5, 1.25, 270),
            new OurPoint(1.3, 1.25, 180),
            new OurPoint(0.98, 0.30, 180),
            new OurPoint(0.82, -0.5, 143),
            new OurPoint(0.5, -0.9, 143)};
    private static double toleranceCenter2 = 110;
    private static double KcCenter2 = 1.5;
    private static double MaxVelocityCenter2 = 1;
    private static double turnSpeedCenter2 = 1.5;
    private static boolean frontCenter2 = true;
    private static Path Center2 = new Path(Center2Points, toleranceCenter2, KcCenter2, MaxVelocityCenter2, turnSpeedCenter2, frontCenter2);

    private static OurPoint[] CenterFoundation2Points = {
            new OurPoint(0.5, -0.9, 143),
            new OurPoint(0.82, -0.5, 143),
            new OurPoint(0.95, 0.15, 180),
            new OurPoint(1.15, 0.5, 180),
            new OurPoint(1.15, 1.05, 180)};
    private static double toleranceCenterFoundation2 = 85;
    private static double KcCenterFoundation2 = 2.5;
    private static double MaxVelocityCenterFoundation2 = 1.5;
    private static double turnSpeedCenterFoundation2 = 0.2;
    private static boolean frontCenterFoundation2 = false;
    private static Path CenterFoundation2 = new Path(CenterFoundation2Points, toleranceCenterFoundation2, KcCenterFoundation2, MaxVelocityCenterFoundation2, turnSpeedCenterFoundation2, frontCenterFoundation2);

    //    TODO Right Points
    private static OurPoint[] Right1Points = {
            new OurPoint(1.566, -0.8325, 143),
            new OurPoint(0.82, -0.3, 143),
            new OurPoint(0.5, -0.7, 143)};
    private static double toleranceRight1 = 50;
    private static double KcRight1 = 0.5;
    private static double MaxVelocityRight1 = 0.6;
    private static double turnSpeedRight1 = 0.4;
    private static boolean frontRight1 = true;
    private static Path Right1 = new Path(Right1Points, toleranceRight1, KcRight1, MaxVelocityRight1, turnSpeedRight1, frontRight1);

    private static OurPoint[] RightFoundationPoints = {
            new OurPoint(0.5, -0.7, 180),
            new OurPoint(0.95, -0.5, 180),
            new OurPoint(0.95, 0.55, 270),
            new OurPoint(0.5, 1.20, 270)};
    private static double toleranceRightFoundation = 44;
    private static double KcRightFoundation = 1.5;
    private static double MaxVelocityRightFoundation = 1.5;
    private static double turnSpeedRightFoundation = 1.1;
    private static boolean frontRightFoundation = false;
    private static Path RightFoundation = new Path(RightFoundationPoints, toleranceRightFoundation, KcRightFoundation, MaxVelocityRightFoundation, turnSpeedRightFoundation, frontRightFoundation);

    private static OurPoint[] Right2Points = {
            new OurPoint(0.5, 1.25, 270),
            new OurPoint(1.3, 1.25, 180),
            new OurPoint(0.93, 0.30, 180),
            new OurPoint(1.05, -0.85, 140),
            new OurPoint(0.48, -1.37, 140)};
    private static double toleranceRight2 = 86;
    private static double KcRight2 = 1.5;
    private static double MaxVelocityRight2 = 1;
    private static double turnSpeedRight2 = 1.5;
    private static boolean frontRight2 = true;
    private static Path Right2 = new Path(Right2Points, toleranceRight2, KcRight2, MaxVelocityRight2, turnSpeedRight2, frontRight2);

    private static OurPoint[] RightFoundation2Points = {
            new OurPoint(0.48, -1.37, 180),
            new OurPoint(0.95, -0.85, 180),
            new OurPoint(0.95, 0.15, 180),
            new OurPoint(1.15, 0.5, 180),
            new OurPoint(1.15, 1, 180),};
    private static double toleranceRightFoundation2 = 85;
    private static double KcRightFoundation2 = 2.5;
    private static double MaxVelocityRightFoundation2 = 1.5;
    private static double turnSpeedRightFoundation2 = 0.2;
    private static boolean frontRightFoundation2 = false;
    private static Path RightFoundation2 = new Path(RightFoundation2Points, toleranceRightFoundation2, KcRightFoundation2, MaxVelocityRightFoundation2, turnSpeedRightFoundation2, frontRightFoundation2);

    //    TODO Parking Points
    private static OurPoint[] ParkingPoints = {
            new OurPoint(1.6, 1, 150),
            new OurPoint(0.8, 0, 150)};
    private static double toleranceParking = 50;
    private static double KcParking = 4.5;
    private static double MaxVelocityParking = 1.5;
    private static double turnSpeedParking = 0.7;
    private static boolean frontParking = true;
    private static Path Parking = new Path(ParkingPoints, toleranceParking, KcParking, MaxVelocityParking, turnSpeedParking, frontParking);

    public static Path[] LeftPaths = {Left1, LeftFoundation, FoundationBack, Left2, LeftFoundation2, Left3, LeftFoundation3, Parking};
    public static Path[] CenterPaths = {Center1, CenterFoundation, Center2, CenterFoundation2, Parking};
    public static Path[] RightPaths = {Right1, RightFoundation, Right2, RightFoundation2, Parking};


}
