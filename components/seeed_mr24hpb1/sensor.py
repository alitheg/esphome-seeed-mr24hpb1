import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID

DEPENDENCIES = ["seeed_mr24hpb1"]

CONF_MOVEMENT_PCT = "movement_pct"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_MOVEMENT_PCT): sensor.sensor_schema(
            unit_of_measurement="%",
            accuracy_decimals=1,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_MOVEMENT_PCT in config:
        sens = await sensor.new_sensor(config[CONF_MOVEMENT_PCT])
        cg.add(hub.set_movement_pct_sensor(sens))
