package org.steelhawks.util.deprecated

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import org.steelhawks.Robot
import org.steelhawks.util.VirtualSubsystem

/**
 * OperatorDashboard.kt
 *
 * This class is used to send data to the iPad operator dashboard.
 * It uses NetworkTables to send data to the Python server running on the driver station.
 * The data is sent in a periodic function.
 * The data is sent every 200ms.
 *
 * @author Farhan Jamil
 */
object OperatorDashboard : VirtualSubsystem() {
    private var ntInstance: NetworkTableInstance = NetworkTableInstance.getDefault()
    private val status: NetworkTable = ntInstance.getTable("status")

    // state values to send to client these SHOULD be updated in periodic
    private val robotState: StringPublisher = status.getStringTopic("robotState").publish()
    private val elevatorLevel: StringPublisher = status.getStringTopic("elevatorLevel").publish()
    private val alliance: StringPublisher = status.getStringTopic("alliance").publish()

//    init {
//        try {
//            val ip = InetAddress.getLocalHost()
//            println("IP ADDR NT4 " + ip.hostAddress + ":2601")
//            DriverStation.reportWarning("IP Address NT4 " + ip.hostAddress + ":2601", false)
//
//            Alert("Operator Dashboard at " + ip.hostAddress + ":2601", AlertType.kInfo).set(true)
//        } catch (e: Exception) {
//            e.printStackTrace()
//        }
//    }

    fun initialize() {
        for (connection in NetworkTableInstance.getDefault().connections) {
            println("Connection: $connection")
            Alert("Connected Devices " + connection.remote_ip, Alert.AlertType.kInfo).set(true)
        }
    }

    private var counter = 0
    override fun periodic() {
        counter = (counter + 1) % 1000

        if (counter % 10 == 0) { // runs this every 10 cycles every 200 ms

            if (DriverStation.getAlliance().isPresent) {
                alliance.set(if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) "Red" else "Blue")
            }

            robotState.set(Robot.getState().name)
        }
    }

    fun close() {
        robotState.close()
        elevatorLevel.close()
        alliance.close()
    }
}