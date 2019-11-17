package com.first1444.frc.robot2019.vision;

@Deprecated
public interface VisionSupplier {
	/**
	 * @param cameraID The ID of the camera to get the latest {@link VisionInstant} from
	 * @return returns the latest {@link VisionInstant} or null if there is none
	 * @throws java.util.NoSuchElementException If there was no {@link VisionInstant} with the given cameraID
	 */
	VisionInstant getInstant(int cameraID);
}
