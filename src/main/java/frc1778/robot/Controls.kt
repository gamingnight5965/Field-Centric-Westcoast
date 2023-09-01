package frc1778.robot

import edu.wpi.first.wpilibj.Joystick
import org.ghrobotics.lib.wrappers.hid.mapControls
import kotlin.math.abs
import kotlin.math.withSign

object Controls {

    val driverControllerGenericHID = Joystick(0)

    val driverController = driverControllerGenericHID.mapControls {  }

    fun handleDeadBand(x: Double, tolerance: Double): Double {
        if (abs(x) < tolerance) {
            return 0.0
        }
        return x.withSign((x - tolerance) / (1.0 - tolerance))
    }
}