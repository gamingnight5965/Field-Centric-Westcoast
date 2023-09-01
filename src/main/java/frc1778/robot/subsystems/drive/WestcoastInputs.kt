package frc1778.robot.subsystems.drive

import com.gamingnight.junction.AutoLog
import frc1778.robot.lib.AbstractWestcoastDriveInputs
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*


@AutoLog
open class WestcoastInputs: AbstractWestcoastDriveInputs {
    override var rightVoltage: SIUnit<Volt> = 0.0.volts
    override var leftVoltage: SIUnit<Volt> = 0.0.volts

    override var rightCurrent: SIUnit<Ampere> = 0.0.amps
    override var leftCurrent: SIUnit<Ampere> = 0.0.amps

    override var rightDistance: SIUnit<Meter> = 0.0.meters

    override var leftDistance: SIUnit<Meter> = 0.0.meters

    override var rightVelocity: SIUnit<LinearVelocity> = SIUnit(0.0)

    override var leftVelocity: SIUnit<LinearVelocity> = SIUnit(0.0)

    override var rightFeedforward: SIUnit<Volt> = 0.0.volts

    override var leftFeedforward: SIUnit<Volt> = 0.0.volts

    override var gyroRaw: SIUnit<Radian> = 0.0.radians

}