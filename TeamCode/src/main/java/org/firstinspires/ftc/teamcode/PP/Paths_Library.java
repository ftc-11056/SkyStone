package org.firstinspires.ftc.teamcode.PP;


public class Paths_Library {


    private static OurPoint[] Left1Points = {
            new OurPoint(1.566, -0.8325, 140),
            new OurPoint(0.85, -0.87, 140),
            new OurPoint(0.61, -1.13, 140)};
    private static double toleranceLeft1 = 50;
    private static double KcLeft1 = 0.5;
    private static double MaxVelocityLeft1 = 0.6;
    private static double turnSpeedLeft1 = 0.4;
    private static boolean frontLeft1 = true;
    private static Path Left1 = new Path(Left1Points, toleranceLeft1, KcLeft1, MaxVelocityLeft1, turnSpeedLeft1, frontLeft1);

    private static OurPoint[] LeftFoundationPoints = {
            new OurPoint(0.68, -1.12, 180),
            new OurPoint(0.95, -0.7, 180),
            new OurPoint(0.95, 0.55, 270),
            new OurPoint(0.55, 1.05, 270)};
    private static double toleranceLeftFoundation = 60;
    private static double KcLeftFoundation = 1.5;
    private static double MaxVelocityLeftFoundation = 1.5;
    private static double turnSpeedLeftFoundation = 1.1;
    private static boolean frontLeftFoundation = false;
    private static Path LeftFoundation = new Path(LeftFoundationPoints, toleranceLeftFoundation, KcLeftFoundation, MaxVelocityLeftFoundation, turnSpeedLeftFoundation, frontLeftFoundation);

    private static OurPoint[] Left2Points = {
            new OurPoint(0.55, 1.05, 270),
            new OurPoint(1.3, 1.05, 200),
            new OurPoint(1.2, 0.30, 200),
            new OurPoint(1.2, -1.6, 155),
            new OurPoint(1.2, -1.75, 155),
            new OurPoint(0.8, -1.75, 155)};
    private static double toleranceLeft2 = 115;
    private static double KcLeft2 = 1.5;
    private static double MaxVelocityLeft2 = 1;
    private static double turnSpeedLeft2 = 1.5;
    private static boolean frontLeft2 = true;
    private static Path Left2 = new Path(Left2Points, toleranceLeft2, KcLeft2, MaxVelocityLeft2, turnSpeedLeft2, frontLeft2);

    private static OurPoint[] LeftFoundation2Points = {
            new OurPoint(0.32, -1.36, 180),
            new OurPoint(-0.9, -0.85, 180),
            new OurPoint(-0.9, 0.15, 180),
            new OurPoint(-1, 0.5, 180),
            new OurPoint(-1, 0.97, 180)};
    private static double toleranceLeftFoundation2 = 80;
    private static double KcLeftFoundation2 = 2.5;
    private static double MaxVelocityLeftFoundation2 = 1.5;
    private static double turnSpeedLeftFoundation2 = 0.2;
    private static boolean frontLeftFoundation2 = false;
    private static Path LeftFoundation2 = new Path(LeftFoundation2Points, toleranceLeftFoundation2, KcLeftFoundation2, MaxVelocityLeftFoundation2, turnSpeedLeftFoundation2, frontLeftFoundation2);

    //    TODO Center Points
    private static OurPoint[] Center1Points = {
            new OurPoint(1.566, -0.8325, 130),
            new OurPoint(0.8, -0.4, 130),
            new OurPoint(0.55, -0.7, 130)};
    private static double toleranceCenter1 = 55;
    private static double KcCenter1 = 0.5;
    private static double MaxVelocityCenter1 = 0.3;
    private static double turnSpeedCenter1 = 0.3;
    private static boolean frontCenter1 = true;
    private static Path Center1 = new Path(Center1Points, toleranceCenter1, KcCenter1, MaxVelocityCenter1, turnSpeedCenter1, frontCenter1);

    private static OurPoint[] CenterFoundationPoints = {
            new OurPoint(0.55, -0.7, 180),
            new OurPoint(0.9, -0.35, 180),
            new OurPoint(0.9, 0.55, 270),
            new OurPoint(0.6, 1.21, 270)};
    private static double toleranceCenterFoundation = 40;
    private static double KcCenterFoundation = 3;
    private static double MaxVelocityCenterFoundation = 1.5;
    private static double turnSpeedCenterFoundation = 1.2;
    private static boolean frontCenterFoundation = false;
    private static Path CenterFoundation = new Path(CenterFoundationPoints, toleranceCenterFoundation, KcCenterFoundation, MaxVelocityCenterFoundation, turnSpeedCenterFoundation, frontCenterFoundation);

    private static OurPoint[] Center2Points = {
            new OurPoint(0.6, 1.18, 270),
            new OurPoint(1.55, 1.18, 180),
            new OurPoint(0.8, 0.3, 180),
            new OurPoint(0.8, -0.85, 130),
            new OurPoint(0.55, -1.1, 130),
            new OurPoint(0.25, -1.1, 130)};
    private static double toleranceCenter2 = 82;
    private static double KcCenter2 = 1.5;
    private static double MaxVelocityCenter2 = 1;
    private static double turnSpeedCenter2 = 1.5;
    private static boolean frontCenter2 = true;
    private static Path Center2 = new Path(Center2Points, toleranceCenter2, KcCenter2, MaxVelocityCenter2, turnSpeedCenter2, frontCenter2);

    private static OurPoint[] CenterFoundation2Points = {
            new OurPoint(0.25, -1.1, 180),
            new OurPoint(0.75, -0.7, 180),
            new OurPoint(0.75, 0.15, 180),
            new OurPoint(1.2, 0.5, 180),
            new OurPoint(1.2, 1.05, 180)};
    private static double toleranceCenterFoundation2 = 80;
    private static double KcCenterFoundation2 = 2.5;
    private static double MaxVelocityCenterFoundation2 = 1.5;
    private static double turnSpeedCenterFoundation2 = 1.5;
    private static boolean frontCenterFoundation2 = false;
    private static Path CenterFoundation2 = new Path(CenterFoundation2Points, toleranceCenterFoundation2, KcCenterFoundation2, MaxVelocityCenterFoundation2, turnSpeedCenterFoundation2, frontCenterFoundation2);

    //    TODO Right Points
    private static OurPoint[] Right1Points = {
            new OurPoint(1.566, -0.8325, 130),
            new OurPoint(0.8, -0.2, 130),
            new OurPoint(0.55, -0.5, 130)};
    private static double toleranceRight1 = 50;
    private static double KcRight1 = 0.5;
    private static double MaxVelocityRight1 = 0.3;
    private static double turnSpeedRight1 = 0.3;
    private static boolean frontRight1 = true;
    private static Path Right1 = new Path(Right1Points, toleranceRight1, KcRight1, MaxVelocityRight1, turnSpeedRight1, frontRight1);

    private static OurPoint[] RightFoundationPoints = {
            new OurPoint(0.55, -0.5, 180),
            new OurPoint(0.9, -0.35, 180),
            new OurPoint(0.9, 0.55, 270),
            new OurPoint(0.6, 1.18, 270)};
    private static double toleranceRightFoundation = 40;
    private static double KcRightFoundation = 3;
    private static double MaxVelocityRightFoundation = 1.5;
    private static double turnSpeedRightFoundation = 1.2;
    private static boolean frontRightFoundation = false;
    private static Path RightFoundation = new Path(RightFoundationPoints, toleranceRightFoundation, KcRightFoundation, MaxVelocityRightFoundation, turnSpeedRightFoundation, frontRightFoundation);

    private static OurPoint[] Right2Points = {
            new OurPoint(0.6, 1.18, 270),
            new OurPoint(1.55, 1.18, 180),
            new OurPoint(0.8, 0.3, 180),
            new OurPoint(0.8, -0.85, 130),
            new OurPoint(0.55, -0.9, 130),
            new OurPoint(0.25, -0.9, 130)};
    private static double toleranceRight2 = 82;
    private static double KcRight2 = 1.5;
    private static double MaxVelocityRight2 = 1;
    private static double turnSpeedRight2 = 1.5;
    private static boolean frontRight2 = true;
    private static Path Right2 = new Path(Right2Points, toleranceRight2, KcRight2, MaxVelocityRight2, turnSpeedRight2, frontRight2);

    private static OurPoint[] RightFoundation2Points = {
            new OurPoint(0.25, -0.9, 180),
            new OurPoint(0.75, -0.6, 180),
            new OurPoint(0.75, 0.15, 180),
            new OurPoint(1.2, 0.5, 180),
            new OurPoint(1.2, 1.05, 180)};
    private static double toleranceRightFoundation2 = 80;
    private static double KcRightFoundation2 = 2.5;
    private static double MaxVelocityRightFoundation2 = 1.5;
    private static double turnSpeedRightFoundation2 = 0.2;
    private static boolean frontRightFoundation2 = false;
    private static Path RightFoundation2 = new Path(RightFoundation2Points, toleranceRightFoundation2, KcRightFoundation2, MaxVelocityRightFoundation2, turnSpeedRightFoundation2, frontRightFoundation2);

    //    TODO Parking Points
    private static OurPoint[] ParkingPoints = {
            new OurPoint(1.2, 0.97, 180),
            new OurPoint(0.8, 0, 180)};
    private static double toleranceParking = 10;
    private static double KcParking = 4.5;
    private static double MaxVelocityParking = 1.5;
    private static double turnSpeedParking = 0.7;
    private static boolean frontParking = true;
    private static Path Parking = new Path(ParkingPoints, toleranceParking, KcParking, MaxVelocityParking, turnSpeedParking, frontParking);

    public static Path[] LeftPaths = {Left1, LeftFoundation, Left2, LeftFoundation2, Parking};
    public static Path[] CenterPaths = {Center1, CenterFoundation, Center2, CenterFoundation2, Parking};
    public static Path[] RightPaths = {Right1, RightFoundation, Right2, RightFoundation2, Parking};


}
