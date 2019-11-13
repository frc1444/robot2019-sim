package com.first1444.frc.robot2019;

import com.first1444.sim.api.RunnableCreator;
import com.first1444.sim.api.frc.FrcDriverStation;
import com.first1444.sim.api.frc.IterativeRobotRunnable;
import com.first1444.sim.wpi.WpiClock;
import com.first1444.sim.wpi.frc.WpiFrcDriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import me.retrodaredevil.controller.wpi.WpiInputCreator;

public class WpiRunnableCreator implements RunnableCreator {
	
	@Override
	public Runnable createRunnable() {
		FrcDriverStation driverStation = new WpiFrcDriverStation(DriverStation.getInstance());
		return new IterativeRobotRunnable(new Robot(
				driverStation, new WpiClock(), new WpiInputCreator(0), new WpiInputCreator(1), new WpiInputCreator(2), new WpiInputCreator(5)
		), driverStation);
	}
	
	@Override
	public void prematureInit() {
	
	}
}
