package com.first1444.frc.util.valuemap.sendable;

import com.first1444.frc.util.valuemap.ValueKey;
import com.first1444.frc.util.valuemap.ValueMap;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import java.util.function.Supplier;

public class ValueMapSendable<T extends Enum<T> & ValueKey> extends SendableBase {

	private final Supplier<ValueMap<T>> valueMapSupplier;

	public ValueMapSendable(Supplier<ValueMap<T>> valueMapSupplier){
		this.valueMapSupplier = valueMapSupplier;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		for(T key : valueMapSupplier.get().getValueKeys()){
			switch(key.getValueType()){
				case DOUBLE:
					builder.addDoubleProperty(key.getName(), () -> valueMapSupplier.get().getDouble(key), null);
					break;
				case STRING:
					builder.addStringProperty(key.getName(), () -> valueMapSupplier.get().getString(key), null);
					break;
				case BOOLEAN:
					builder.addBooleanProperty(key.getName(), () -> valueMapSupplier.get().getBoolean(key), null);
					break;
				default:
					throw new UnsupportedOperationException("Unsupported value type: " + key.getValueType());
			}
		}
	}
}
