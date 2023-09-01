package frc1778.robot

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase.isReal
import frc1778.robot.subsystems.drive.Drive
import org.ghrobotics.lib.wrappers.LoggedFalconTimedRobot
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGWriter

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * This is written as an object rather than a class since there should only ever be a single instance, and
 * it cannot take any constructor arguments. This makes it a natural fit to be an object in Kotlin.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : LoggedFalconTimedRobot()
{
    override fun robotInit() {

        Logger.getInstance().recordMetadata("ProjectName", "Cold Fusion") // Set a metadata value

        if (LoggedRobot.isReal()) {
            Logger.getInstance().addDataReceiver(WPILOGWriter("/media/sda1/")) // Log to a USB stick
            Logger.getInstance().addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
            PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging
        } else {
//            setUseTiming(false) // Run as fast as possible
//            val logPath = LogFileUtil.findReplayLog() // Pull the replay log from AdvantageScope (or prompt the user)
//            Logger.getInstance().setReplaySource(WPILOGReader(logPath)) // Read replay log
//            Logger.getInstance()
//                .addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))) // Save outputs to a new log

            Logger.getInstance().addDataReceiver(NT4Publisher())
        }
        +Drive
        Logger.getInstance()
            .start()
    }



    override fun robotPeriodic() {}

    override fun autonomousInit() {}

    override fun autonomousPeriodic() {}

    override fun teleopInit() {}

    override fun teleopPeriodic() {}

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}