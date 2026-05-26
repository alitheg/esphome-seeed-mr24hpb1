import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID, seeed_mr24hpb1_ns

DEPENDENCIES = ["seeed_mr24hpb1"]

RebootButton = seeed_mr24hpb1_ns.class_("RebootButton", button.Button)

CONF_REBOOT = "reboot"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_REBOOT): button.button_schema(RebootButton),
    }
)


async def to_code(config):
    if CONF_REBOOT in config:
        btn = await button.new_button(config[CONF_REBOOT])
        await cg.register_parented(btn, config[CONF_SEEED_MR24HPB1_ID])
