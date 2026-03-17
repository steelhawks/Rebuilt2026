package org.steelhawks.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.steelhawks.Autos;
import org.steelhawks.ChoreoTraj;

public class RRebound extends SequentialCommandGroup {

	public RRebound() {
		super(
		Autos.followTrajectory(ChoreoTraj.LRebound.);
		);
		withName("Hi");
	}
}
