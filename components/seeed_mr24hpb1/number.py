import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_MODE

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID, seeed_mr24hpb1_ns

DEPENDENCIES = ["seeed_mr24hpb1"]

SensitivityNumber = seeed_mr24hpb1_ns.class_("SensitivityNumber", number.Number)

CONF_SENSITIVITY = "sensitivity"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_SENSITIVITY): number.number_schema(SensitivityNumber).extend(
            {cv.Optional(CONF_MODE, default="SLIDER"): cv.enum(number.NUMBER_MODES, upper=True)}
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_SENSITIVITY in config:
        num = await number.new_number(
            config[CONF_SENSITIVITY], min_value=1, max_value=3, step=1
        )
        await cg.register_parented(num, config[CONF_SEEED_MR24HPB1_ID])
        cg.add(hub.set_sensitivity_number(num))
