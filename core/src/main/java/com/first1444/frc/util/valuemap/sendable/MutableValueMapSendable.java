package com.first1444.frc.util.valuemap.sendable;

import com.first1444.frc.util.valuemap.MutableValueMap;
import com.first1444.frc.util.valuemap.ValueKey;
import com.first1444.frc.util.valuemap.ValueMap;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class MutableValueMapSendable<T extends Enum<T> & ValueKey> extends SendableBase {

	private final MutableValueMap<T> valueMap;

	public MutableValueMapSendable(Class<T> clazz){
		valueMap = new MutableValueMap<>(clazz);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("RobotPreferences");
		for(T key : valueMap.getValueKeys()){
			switch(key.getValueType()){
				case DOUBLE:
					builder.addDoubleProperty(key.getName(), () -> valueMap.getDouble(key), (value) -> valueMap.setDouble(key, value));
					break;
				case STRING:
					builder.addStringProperty(key.getName(), () -> valueMap.getString(key), (value) -> valueMap.setString(key, value));
					break;
				case BOOLEAN:
					builder.addBooleanProperty(key.getName(), () -> valueMap.getBoolean(key), (value) -> valueMap.setBoolean(key, value));
					break;
				default:
					throw new UnsupportedOperationException("Unsupported value type: " + key.getValueType());
			}
		}
	}
	public MutableValueMap<T> getMutableValueMap(){
		return valueMap;
	}
	public ValueMap<T> getImmutableValueMap(){
		return valueMap.build();
	}
}
