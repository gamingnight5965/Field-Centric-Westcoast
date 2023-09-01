package frc1778.robot.subsystems.drive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.TalonFXControlMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.ctre.phoenix.sensors.Pigeon2
import com.ctre.phoenix.sensors.WPI_Pigeon2
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import frc1778.robot.lib.AbstractWestcoastDriveInputs
import frc1778.robot.lib.WestcoastDriveIO
import org.apache.commons.math3.analysis.function.Log
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.LinearVelocity
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.motors.ctre.falconFX
import org.ghrobotics.lib.utils.Source
import org.littletonrobotics.junction.Logger
import kotlin.math.roundToInt

class WestcoastSimulationIO : WestcoastDriveIO {

    val lengthModel = NativeUnitLengthModel(17000.nativeUnits, 3.inches)


    val rightMain = falconFX(WPI_TalonFX(1), lengthModel) {
        talonFX.configFactoryDefault()
        brakeMode = true
    }

    val leftMain = falconFX(WPI_TalonFX(2), lengthModel) {
        talonFX.configFactoryDefault()
        brakeMode = true
    }

    val rightFollower = falconFX(WPI_TalonFX(3), lengthModel) {
        talonFX.configFactoryDefault()
        brakeMode = true
        follow(rightMain)
    }

    val leftFollower = falconFX(WPI_TalonFX(4), lengthModel) {
        talonFX.configFactoryDefault()
        brakeMode = true
        follow(leftMain)
    }

    private var wheelSpeeds = DifferentialDriveWheelSpeeds(0.0, 0.0)

    val rightMainSimulationCollection = (rightMain.talonFX as WPI_TalonFX).simCollection
    val leftMainSimulationCollection = (leftMain.talonFX as WPI_TalonFX).simCollection


    val WPI_Pigeon2 = WPI_Pigeon2(0)

    val pigeonSimCollection = WPI_Pigeon2.simCollection

    val driveSim = DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2),
        1/((9.0 / 50) * (28.0 / 42)),
        3.329 ,
        39.91613 ,
        3.inches.value,
        23.inches.value,
        null
    )


    override fun <T : AbstractWestcoastDriveInputs> updateInputs(inputs: T) {
        rightMainSimulationCollection.setBusVoltage(RobotController.getBatteryVoltage())
        leftMainSimulationCollection.setBusVoltage(RobotController.getBatteryVoltage())


        driveSim.setInputs(
            leftMainSimulationCollection.motorOutputLeadVoltage,
            rightMainSimulationCollection.motorOutputLeadVoltage
        )


        driveSim.update(0.02)

        rightMainSimulationCollection.setIntegratedSensorRawPosition(
            lengthModel.toNativeUnitPosition(driveSim.rightPositionMeters.meters).value.roundToInt()
        )

        rightMainSimulationCollection.setIntegratedSensorVelocity(
            lengthModel.toNativeUnitVelocity(SIUnit(driveSim.rightVelocityMetersPerSecond)).value.roundToInt()
        )

        leftMainSimulationCollection.setIntegratedSensorRawPosition(
            lengthModel.toNativeUnitPosition(driveSim.leftPositionMeters.meters).value.roundToInt()
        )

        leftMainSimulationCollection.setIntegratedSensorVelocity(
            lengthModel.toNativeUnitVelocity(SIUnit(driveSim.leftVelocityMetersPerSecond)).value.roundToInt()
        )

        pigeonSimCollection.setRawHeading(driveSim.heading.degrees)

        with(inputs) {
            gyroRaw = WPI_Pigeon2.yaw.degrees

            this.leftVoltage = leftMainSimulationCollection.motorOutputLeadVoltage.volts
            this.rightVoltage = rightMainSimulationCollection.motorOutputLeadVoltage.volts

            leftCurrent = leftMain.drawnCurrent
            rightCurrent = rightMain.drawnCurrent

            leftVelocity = leftMain.encoder.velocity
            rightVelocity = rightMain.encoder.velocity

            leftDistance =
                lengthModel.fromNativeUnitPosition((leftMain.talonFX as WPI_TalonFX).getSelectedSensorPosition().nativeUnits)
            rightDistance =
                lengthModel.fromNativeUnitPosition((rightMain.talonFX as WPI_TalonFX).getSelectedSensorPosition().nativeUnits)
        }


    }

    override fun setVoltages(leftVoltage: SIUnit<Volt>, rightVoltage: SIUnit<Volt>) {
        leftMain.setVoltage(leftVoltage)
        rightMain.setVoltage(rightVoltage)
    }

    override fun setOutputs(leftPercent: Double, rightPercent: Double) {
        leftMain.setDutyCycle(leftPercent)
        rightMain.setDutyCycle(rightPercent)
    }

    override fun setWheelSpeeds(wheelSpeeds: DifferentialDriveWheelSpeeds, maxAchievableSpeed: SIUnit<LinearVelocity>) {
        this.wheelSpeeds = wheelSpeeds
        setVoltages(
            (wheelSpeeds.leftMetersPerSecond / maxAchievableSpeed.value).volts * 12.0,
            (wheelSpeeds.rightMetersPerSecond / maxAchievableSpeed.value).volts * 12.0,
        )
    }

    override fun setVelocity(
        leftVelocity: Double, rightVelocity: Double, leftAcceleration: Double, rightAcceleration: Double
    ) {
        throw NotImplementedError()
    }

    override fun setVelocity(leftVelocity: SIUnit<LinearVelocity>, rightVelocity: SIUnit<LinearVelocity>) {
        throw NotImplementedError()
    }

    override fun setNeutral() {
        leftMain.setNeutral()
        rightMain.setNeutral()
    }

    override val gyro: Source<Rotation2d> = { WPI_Pigeon2.rotation2d }
}