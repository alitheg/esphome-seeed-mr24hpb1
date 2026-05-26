import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from . import MR24HPB1, CONF_SEEED_MR24HPB1_ID, seeed_mr24hpb1_ns

DEPENDENCIES = ["seeed_mr24hpb1"]

SceneSelect = seeed_mr24hpb1_ns.class_("SceneSelect", select.Select)

CONF_SCENE_MODE = "scene_mode"
SCENE_OPTIONS = [
    "Default",
    "Area Detection",
    "Bathroom",
    "Bedroom",
    "Living Room",
    "Office",
    "Hotel",
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SEEED_MR24HPB1_ID): cv.use_id(MR24HPB1),
        cv.Optional(CONF_SCENE_MODE): select.select_schema(SceneSelect),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_SEEED_MR24HPB1_ID])
    if CONF_SCENE_MODE in config:
        sel = await select.new_select(config[CONF_SCENE_MODE], options=SCENE_OPTIONS)
        await cg.register_parented(sel, config[CONF_SEEED_MR24HPB1_ID])
        cg.add(hub.set_scene_select(sel))
