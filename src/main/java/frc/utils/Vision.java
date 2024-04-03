package frc.utils;

import java.util.function.BooleanSupplier;

// import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

public final class Vision {

	public static final PubSubOption NT_ENTRY_DEFAULT = PubSubOption.periodic(0.02);
	public static final PubSubOption[] NT_DATA_SUBSCRIBER = new PubSubOption[] { NT_ENTRY_DEFAULT,
			PubSubOption.keepDuplicates(true),
			PubSubOption.pollStorage(10)
	};

	static NetworkTable table;
	static DoubleArrayEntry x;
	static DoubleArrayEntry y;
	static DoubleArrayEntry z;
	static IntegerArrayEntry ids;
	static DoubleArrayEntry rx;
	public static void setupVision() {
		System.out.println("setup vision");
		double[] defaultDoubleArray = { 0 };
		long[] defaultLongArray = { 0 };
		table = NetworkTableInstance.getDefault().getTable("/Vision Server/Pipelines/bv2024");
		x = table.getDoubleArrayTopic("x")
				.getEntry(defaultDoubleArray, Vision.NT_ENTRY_DEFAULT);
		y = table.getDoubleArrayTopic("y")
				.getEntry(defaultDoubleArray, Vision.NT_ENTRY_DEFAULT);
		z = table.getDoubleArrayTopic("z")
				.getEntry(defaultDoubleArray, Vision.NT_ENTRY_DEFAULT);
		ids = table.getIntegerArrayTopic("ids")
				.getEntry(defaultLongArray, Vision.NT_ENTRY_DEFAULT);
		rx = table.getDoubleArrayTopic("rx")
		.getEntry(defaultDoubleArray, Vision.NT_ENTRY_DEFAULT);

	}

	private static final double visionYawConstant = 1;

	public static void fuseVision(DriveSubsystem db, boolean filter_bounds) {
		long[] idArray = ids.get().clone();
		double[] zArray = z.get().clone();
		double[] targetYaw = rx.get().clone();
		if (idArray.length > 0 && zArray.length == idArray.length && targetYaw.length == idArray.length) {
			int iteration = 0;
			for (long value : idArray) {
				if (value <= 16 && value >= 1) {
					db.applyVisionUpdate(
							value,
							new Translation3d(zArray[iteration],
									new Rotation3d(0, 0, targetYaw[iteration]*visionYawConstant)),
							System.currentTimeMillis());
					iteration++;
				}
			}
		}
	}
}
