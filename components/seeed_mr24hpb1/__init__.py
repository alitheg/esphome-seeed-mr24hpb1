import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CONF_UART_ID = "uart_id"

seeed_mr24hpb1_ns = cg.esphome_ns.namespace("seeed_mr24hpb1")
MR24HPB1 = seeed_mr24hpb1_ns.class_("MR24HPB1", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MR24HPB1),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    # 1. Resolve the UART component instance
    uart_var = yield cg.get_variable(config[CONF_UART_ID])

    # 2. Create the radar component using the UART component
    var = cg.new_Pvariable(config[CONF_ID], uart_var)

    # 3. Register the radar as a component
    yield cg.register_component(var, config)

    # 4. Register the UART device using the **ID from config**, not the resolved var
    yield uart.register_uart_device(var, config[CONF_UART_ID])