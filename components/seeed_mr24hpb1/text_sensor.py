import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID

DEPENDENCIES = ["seeed_mr24hpb1"]

CONF_MOVEMENT_CLASS = "movement_class"
CONF_DEVICE_ID = "device_id"
CONF_SOFTWARE_VERSION = "software_version"
CONF_HARDWARE_VERSION = "hardware_version"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_MOVEMENT_CLASS): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_DEVICE_ID): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SOFTWARE_VERSION): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_HARDWARE_VERSION): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_MOVEMENT_CLASS in config:
        s = await text_sensor.new_text_sensor(config[CONF_MOVEMENT_CLASS])
        cg.add(hub.set_movement_class_text_sensor(s))
    if CONF_DEVICE_ID in config:
        s = await text_sensor.new_text_sensor(config[CONF_DEVICE_ID])
        cg.add(hub.set_device_id_text_sensor(s))
    if CONF_SOFTWARE_VERSION in config:
        s = await text_sensor.new_text_sensor(config[CONF_SOFTWARE_VERSION])
        cg.add(hub.set_software_version_text_sensor(s))
    if CONF_HARDWARE_VERSION in config:
        s = await text_sensor.new_text_sensor(config[CONF_HARDWARE_VERSION])
        cg.add(hub.set_hardware_version_text_sensor(s))
