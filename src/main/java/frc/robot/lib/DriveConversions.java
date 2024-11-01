package frc.robot.lib;

public class DriveConversions {
    
    public static double WheelRPMToMotorRPS(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio; // Takes gear ratio into account
        double motorRPS = motorRPM / 60; // 60 seconds in a minute
        return motorRPS;
    }

    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference); // 60 seconds in a minute, divide by circumference 
        double motorVelocity = WheelRPMToMotorRPS(wheelRPM, gearRatio);
        return motorVelocity;
    }

    public static double toMPS(double RPS, double circumference) {
        double MPS = RPS * circumference;
        return MPS;
    }

    public static double rotationsToMeters(double wheelRotations, double circumference){
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }
}
