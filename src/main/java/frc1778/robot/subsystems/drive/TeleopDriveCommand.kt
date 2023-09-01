package frc1778.robot.subsystems.drive

import frc1778.robot.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.utils.DoubleSource
import org.ghrobotics.lib.utils.withDeadband
import kotlin.math.abs

class TeleopDriveCommand : FalconCommand(Drive) {

    override fun execute() {
        val (tx, ty) = translations()

        if (abs(tx) + abs(ty) > 0) {
            Drive.fieldCentricDrive(
                tx, ty, trigger()
            )
        } else {
            val (rx, ry) = rotations()
            Drive.pointInPlace(rx, ry)
        }
//        val (rx, ry) = rotations()
//        Drive.tankDriveVoltage(12.0.volts * tx, 12.0.volts * rx)
    }


    companion object {
        private const val DEADBAND = 0.125

        val translationX = (-Controls.driverController.getRawAxis(1)).withDeadband(DEADBAND)
        val translationY = (-Controls.driverController.getRawAxis(0)).withDeadband(DEADBAND)
        val rotationX = (-Controls.driverController.getRawAxis(5)).withDeadband(DEADBAND)
        val rotationY = (-Controls.driverController.getRawAxis(4)).withDeadband(DEADBAND)

        val reverse = Controls.driverController.getRawButton(1)

        fun translations(): Pair<Double, Double> {
            return translationX() to translationY()
        }

        fun rotations(): Pair<Double, Double> {
            return rotationX() to rotationY()
        }

        fun trigger(): Double {
            return Controls.leftTrigger() * -1 + Controls.rightTrigger()
        }

        operator fun DoubleSource.unaryMinus(): DoubleSource = { -this@unaryMinus() }

    }
}