package org.steelhawks.util

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.button.Trigger

class DashboardTrigger(private val entry: String) : Trigger({
    val table = NetworkTableInstance.getDefault().getTable("controls")
    val retrievedEntry = table.getEntry(entry)
    retrievedEntry.getBoolean(false)
})
