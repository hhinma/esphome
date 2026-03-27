import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_TYPE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_POWER,
)

from . import ups_hid_ns, UpsHidComponent, CONF_UPS_HID_ID

DEPENDENCIES = ["ups_hid"]

UpsHidBinarySensor = ups_hid_ns.class_(
    "UpsHidBinarySensor", binary_sensor.BinarySensor, cg.Component
)

BINARY_SENSOR_TYPES = {
    "online": {
        # DEVICE_CLASS_POWER: ON="Power detected" (mains present),
        # OFF="No power" (mains absent / on battery).
        # DEVICE_CLASS_CONNECTIVITY was previously used but produced the
        # misleading label "Disconnected" which users confused with the USB
        # device going offline rather than mains AC being absent.
        "device_class": DEVICE_CLASS_POWER,
    },
    "on_battery": {
        # DEVICE_CLASS_PROBLEM: ON="Problem" (running on battery, no mains),
        # OFF="OK" (on mains, normal operation).
        # DEVICE_CLASS_BATTERY was previously used but that class represents
        # a LOW BATTERY charge warning (ON="Low", OFF="Normal") which is both
        # semantically wrong and confusing alongside the low_battery sensor.
        "device_class": DEVICE_CLASS_PROBLEM,
    },
    "low_battery": {
        "device_class": DEVICE_CLASS_BATTERY,
    },
    "fault": {
        "device_class": DEVICE_CLASS_PROBLEM,
    },
    "overload": {
        "device_class": DEVICE_CLASS_POWER,
    },
    "charging": {
        "device_class": DEVICE_CLASS_BATTERY,
    },
}


CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(UpsHidBinarySensor).extend(
    {
        cv.GenerateID(CONF_UPS_HID_ID): cv.use_id(UpsHidComponent),
        cv.Required(CONF_TYPE): cv.one_of(*BINARY_SENSOR_TYPES, lower=True),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_UPS_HID_ID])
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)

    sensor_type = config[CONF_TYPE]
    cg.add(var.set_sensor_type(sensor_type))
    cg.add(parent.register_binary_sensor(var, sensor_type))

    # Apply sensor type specific configuration
    if sensor_type in BINARY_SENSOR_TYPES:
        sensor_config = BINARY_SENSOR_TYPES[sensor_type]

        # Override config with sensor type defaults if not specified
        if "device_class" not in config and "device_class" in sensor_config:
            cg.add(var.set_device_class(sensor_config["device_class"]))
