import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID

DEPENDENCIES = ["seeed_mr24hpb1"]

CONF_PRESENCE = "presence"
CONF_MOTION = "motion"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
            device_class="occupancy"
        ),
        cv.Optional(CONF_MOTION): binary_sensor.binary_sensor_schema(
            device_class="motion"
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_PRESENCE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_PRESENCE])
        cg.add(hub.set_presence_binary_sensor(sens))
    if CONF_MOTION in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_MOTION])
        cg.add(hub.set_motion_binary_sensor(sens))
