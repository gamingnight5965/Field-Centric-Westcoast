package frc1778.robot.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
import org.ghrobotics.lib.mathematics.units.Ampere
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.subsystems.SensorlessCompatibleSubsystem
import org.ghrobotics.lib.subsystems.drive.FalconDriveHelper
import org.ghrobotics.lib.subsystems.drive.westcoast.TrajectoryTrackerWestCoastDriveBase
import org.ghrobotics.lib.subsystems.drive.westcoast.WestCoastTrajectoryTrackerCommand
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map

abstract class AdvantageFalconWestcoastDrivetrain : TrajectoryTrackerWestCoastDriveBase(),
    SensorlessCompatibleSubsystem {

    abstract val westcoastDriveIO: WestcoastDriveIO
    abstract val westcoastDriveInputs: AbstractWestcoastDriveInputs

    protected val driveHelper = FalconDriveHelper()

    abstract val trackWidth: Double
    abstract val maxSpeed: SIUnit<LinearVelocity>

    override var robotPosition: Pose2d = Pose2d()

    abstract fun resetPose(pose: Pose2d)

    override fun setNeutral() {
        westcoastDriveIO.setNeutral()
    }

    override fun setOutputSI(
        leftVelocity: Double, rightVelocity: Double, leftAcceleration: Double, rightAcceleration: Double
    ) {
        westcoastDriveIO.setVelocity(
            leftVelocity, rightVelocity, leftAcceleration, rightAcceleration
        )
    }

    fun followTrajectory(trajectory: Trajectory, mirrored: Boolean = false) =
        WestCoastTrajectoryTrackerCommand(this, Source(if (mirrored) trajectory.mirror() else trajectory))

    fun followTrajectory(trajectory: Trajectory, mirrored: BooleanSource) =
        WestCoastTrajectoryTrackerCommand(this, mirrored.map(trajectory.mirror(), trajectory))

    fun followTrajectory(trajectory: Source<Trajectory>) =
        WestCoastTrajectoryTrackerCommand(this, trajectory)
}

interface WestcoastDriveIO {

    fun <T : AbstractWestcoastDriveInputs> updateInputs(inputs: T)

    fun setVoltages(leftVoltage: SIUnit<Volt>, rightVoltage: SIUnit<Volt>)
    fun setOutputs(leftPercent: Double, rightPercent: Double)
    fun setVoltages(volts: Pair<SIUnit<Volt>, SIUnit<Volt>>) = setVoltages(volts.first, volts.second)
    fun setOutputs(percents: Pair<Double, Double>) = setOutputs(percents.first, percents.second)
    fun setWheelSpeeds(wheelSpeeds: DifferentialDriveWheelSpeeds, maxAchievableSpeed: SIUnit<LinearVelocity>)

    fun setVelocity(leftVelocity: Double, rightVelocity: Double, leftAcceleration: Double, rightAcceleration: Double)
    fun setVelocity(leftVelocity: SIUnit<LinearVelocity>, rightVelocity: SIUnit<LinearVelocity>)
    fun setVelocity(velocity: Pair<SIUnit<LinearVelocity>, SIUnit<LinearVelocity>>) =
        setVelocity(velocity.first, velocity.second)


    fun setNeutral()

    val gyro: Source<Rotation2d>

}

interface AbstractWestcoastDriveInputs {
    var rightVoltage: SIUnit<Volt>
    var leftVoltage: SIUnit<Volt>

    var rightCurrent: SIUnit<Ampere>
    var leftCurrent: SIUnit<Ampere>

    var rightDistance: SIUnit<Meter>
    var leftDistance: SIUnit<Meter>

    var rightVelocity: SIUnit<LinearVelocity>
    var leftVelocity: SIUnit<LinearVelocity>

    var rightFeedforward: SIUnit<Volt>
    var leftFeedforward: SIUnit<Volt>

    var gyroRaw: SIUnit<Radian>
}