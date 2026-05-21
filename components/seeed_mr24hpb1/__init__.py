import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart, binary_sensor, sensor, text_sensor
from esphome.const import CONF_ID

import logging

AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]

CONF_PRESENCE_SENSOR = "presence_sensor"
CONF_MOTION_SENSOR = "motion_sensor"
CONF_MOVEMENT_PCT_SENSOR = "movement_pct_sensor"
CONF_THRESHOLD_GEAR_SENSOR = "threshold_gear_sensor"
CONF_MOVEMENT_CLASS_SENSOR = "movement_class_sensor"
CONF_DEVICE_ID_SENSOR = "device_id_sensor"
CONF_SOFTWARE_VERSION_SENSOR = "software_version_sensor"
CONF_HARDWARE_VERSION_SENSOR = "hardware_version_sensor"
CONF_SCENE_MODE_SENSOR = "scene_mode_sensor"

seeed_mr24hpb1_ns = cg.esphome_ns.namespace("seeed_mr24hpb1")
MR24HPB1Component = seeed_mr24hpb1_ns.class_("MR24HPB1", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MR24HPB1Component),
        cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
        # All sensors are optional - wire up only the ones you want exposed.
        cv.Optional(CONF_PRESENCE_SENSOR): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_MOTION_SENSOR): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_MOVEMENT_PCT_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_THRESHOLD_GEAR_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_MOVEMENT_CLASS_SENSOR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_DEVICE_ID_SENSOR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_SOFTWARE_VERSION_SENSOR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_HARDWARE_VERSION_SENSOR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_SCENE_MODE_SENSOR): cv.use_id(text_sensor.TextSensor),
    }
)
CONFIG_SCHEMA = CONFIG_SCHEMA.extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    logging.getLogger().info("SEEED MR24HPB1 to_code start")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Resolve and assign a sensor only if it was configured.
    async def set_sensor(name, setter):
        if name not in config:
            return
        sensor_var = await cg.get_variable(config[name])
        cg.add(setter(sensor_var))

    await set_sensor(CONF_PRESENCE_SENSOR, var.set_presence_sensor)
    await set_sensor(CONF_MOTION_SENSOR, var.set_motion_sensor)
    await set_sensor(CONF_MOVEMENT_PCT_SENSOR, var.set_movement_pct_sensor)
    await set_sensor(CONF_THRESHOLD_GEAR_SENSOR, var.set_threshold_gear_sensor)
    await set_sensor(CONF_MOVEMENT_CLASS_SENSOR, var.set_movement_class_sensor)
    await set_sensor(CONF_DEVICE_ID_SENSOR, var.set_device_id_sensor)
    await set_sensor(CONF_SOFTWARE_VERSION_SENSOR, var.set_software_version_sensor)
    await set_sensor(CONF_HARDWARE_VERSION_SENSOR, var.set_hardware_version_sensor)
    await set_sensor(CONF_SCENE_MODE_SENSOR, var.set_scene_mode_sensor)