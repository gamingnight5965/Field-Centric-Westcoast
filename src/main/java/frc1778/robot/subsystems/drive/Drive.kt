package frc1778.robot.subsystems.drive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.controller.RamseteController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc1778.robot.lib.AdvantageFalconWestcoastDrivetrain
import frc1778.robot.lib.WestcoastDriveIO
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.inches
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import kotlin.math.*

object Drive : AdvantageFalconWestcoastDrivetrain() {
    const val driveGearRatio = (9.0/50) * (28.0/42)

    override val westcoastDriveIO: WestcoastDriveIO = WestcoastSimulationIO()
    override val westcoastDriveInputs: WestcoastInputsAutoLogged = WestcoastInputsAutoLogged()
    override val trackWidth: Double = 26.inches.value

    override val maxSpeed: SIUnit<LinearVelocity> = SIUnit(DCMotor.getFalcon500(2).freeSpeedRadPerSec * driveGearRatio * 3.inches.value)

    val odometry = DifferentialDriveOdometry(
        westcoastDriveIO.gyro(), westcoastDriveInputs.leftDistance.value, westcoastDriveInputs.rightDistance.value
    )

    init {
        defaultCommand = TeleopDriveCommand()
    }


    override fun periodic() {
        Logger.getInstance().processInputs("Drive Inputs", westcoastDriveInputs)
        robotPosition = odometry.update(
            westcoastDriveIO.gyro(), westcoastDriveInputs.leftDistance.value, westcoastDriveInputs.rightDistance.value
        )
        westcoastDriveIO.updateInputs(westcoastDriveInputs)
        Logger.getInstance().recordOutput("Drive Position", robotPosition)
    }

    override fun resetPose(pose: Pose2d) {
        odometry.resetPosition(
            westcoastDriveIO.gyro(),
            westcoastDriveInputs.leftDistance.value,
            westcoastDriveInputs.rightDistance.value,
            pose
        )

    }

    override val controller: RamseteController = RamseteController(2.5, 1.1)
    override val kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(trackWidth)

    override fun disableClosedLoopControl() {
        TODO("Not yet implemented")
    }

    override fun enableClosedLoopControl() {
        TODO("Not yet implemented")
    }


    private val thetaController = ProfiledPIDController(
        4.0, 0.075, 0.125, TrapezoidProfile.Constraints(32 * PI, 128 * PI)
    ).apply {
        enableContinuousInput(-PI, PI)
        setTolerance(0.005)
    }

    fun fieldCentricDrive(x: Double, y: Double, reverse: Boolean = false) {
        val desiredHeading = getDesiredHeading(x, y, reverse)
        lastDesiredHeading = desiredHeading

        val goalPose = odometry.poseMeters.plus(
            Transform2d(
                Translation2d(
                    sqrt(x.pow(2) + y.pow(2)) * maxSpeed.value * 0.125,
                    Rotation2d.fromRadians(desiredHeading).minus(odometry.poseMeters.rotation)
                ),
                Rotation2d.fromRadians(desiredHeading).minus(odometry.poseMeters.rotation).apply {
                    if (reverse) {
                        rotateBy(Rotation2d.fromDegrees(180.0))
                    }
                }
            )
        )
        Logger.getInstance().recordOutput("Drive Goal Pose", goalPose)

        val chassisSpeeds = controller.calculate(
            odometry.poseMeters,
            goalPose,
            sqrt(x.pow(2) + y.pow(2)) * maxSpeed.value * if(reverse) -1.0 else 1.0,
            0.0
        )


        val wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds)
        wheelSpeeds.desaturate(maxSpeed.value)

        westcoastDriveIO.setWheelSpeeds(wheelSpeeds, maxSpeed)
    }

    fun pointInPlace(x: Double, y: Double) {
        val desiredHeading = getDesiredHeading(x, y, false)
        lastDesiredHeading = desiredHeading

        val goalPose = Pose2d(
            odometry.poseMeters.translation,
            Rotation2d.fromRadians(desiredHeading)
        )

        Logger.getInstance().recordOutput("Drive Goal Pose", goalPose)

        val chassisSpeeds = ChassisSpeeds(
            0.0,
            0.0,
            thetaController.calculate(odometry.poseMeters.rotation.radians, desiredHeading)
        )

        val wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds)
        wheelSpeeds.desaturate(maxSpeed.value)

        westcoastDriveIO.setWheelSpeeds(wheelSpeeds, maxSpeed)
    }

    fun tankDriveVoltage(leftVolts: SIUnit<Volt>, rightVolts: SIUnit<Volt>) {
        westcoastDriveIO.setVoltages(leftVolts, rightVolts)
    }


    private var lastDesiredHeading = 0.0
    fun getDesiredHeading(x: Double, y: Double, reverse: Boolean): Double {
//        return if (x == 0.0 && y == 0.0) lastDesiredHeading else atan2(y, x)
        return MathUtil.angleModulus(atan2(y, x) + if (reverse) PI else 0.0)
    }


}