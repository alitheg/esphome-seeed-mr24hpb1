import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@alitheg"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor", "select", "number", "button"]

CONF_SEEED_MR24HPB1_ID = "seeed_mr24hpb1_id"

seeed_mr24hpb1_ns = cg.esphome_ns.namespace("seeed_mr24hpb1")
MR24HPB1 = seeed_mr24hpb1_ns.class_("MR24HPB1", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(MR24HPB1)})
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
