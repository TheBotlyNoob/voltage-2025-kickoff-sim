// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** IO implementation for maple-sim. */
public class GyroIOSim implements GyroIO {

  private final GyroSimulation gyroSim;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSim = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;

    inputs.yawPosition = gyroSim.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSim.getMeasuredAngularVelocity().in(RadiansPerSecond);

    inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryYawPositions = gyroSim.getCachedGyroReadings();
  }
}
