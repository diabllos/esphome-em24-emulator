"""ESPHome external component: emulate a Carlo Gavazzi EM24 grid meter over Modbus TCP.

Presents local sensors (power / import / export energy, optional voltage / frequency)
to a Victron GX device as a native Carlo Gavazzi EM24 Ethernet energy meter.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID

CODEOWNERS = ["@local"]
DEPENDENCIES = ["network"]

em24_ns = cg.esphome_ns.namespace("em24_meter")
EM24Meter = em24_ns.class_("EM24Meter", cg.Component)

CONF_POWER_L1 = "power_L1"
CONF_POWER_L2 = "power_L2"
CONF_POWER_L3 = "power_L3"
CONF_IMPORT_ENERGY = "import_energy"
CONF_EXPORT_ENERGY = "export_energy"
CONF_VOLTAGE_L1 = "voltage_L1"
CONF_VOLTAGE_L2 = "voltage_L2"
CONF_VOLTAGE_L3 = "voltage_L3"
CONF_FREQUENCY = "frequency"
CONF_PORT = "port"
CONF_UNIT_ID = "unit_id"
CONF_PHASES = "phases"
CONF_SERIAL = "serial"
CONF_INVERT_POWER = "invert_power"

# Maps to Victron's PhaseConfig register (0x1002) index -> nr of phases
#   3 -> "1P"  (1 phase)        <- default, correct for total-only SML data
#   0 -> "3P.n"(3 phase + N)
#   4 -> "3P"  (3 phase)
PHASE_MAP = {"1P": 3, "3P.n": 0, "3P": 4}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(EM24Meter),
        cv.Required(CONF_POWER_L1): cv.use_id(sensor.Sensor),
        cv.Required(CONF_POWER_L2): cv.use_id(sensor.Sensor),
        cv.Required(CONF_POWER_L3): cv.use_id(sensor.Sensor),
        cv.Required(CONF_IMPORT_ENERGY): cv.use_id(sensor.Sensor),
        cv.Required(CONF_EXPORT_ENERGY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_VOLTAGE_L1): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_VOLTAGE_L2): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_VOLTAGE_L3): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_FREQUENCY): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_PORT, default=502): cv.port,
        cv.Optional(CONF_UNIT_ID, default=1): cv.int_range(min=1, max=247),
        cv.Optional(CONF_PHASES, default="1P"): cv.enum(PHASE_MAP, upper=False),
        cv.Optional(CONF_SERIAL, default="EM24EMU0000001"): cv.All(
            cv.string, cv.Length(max=14)
        ),
        cv.Optional(CONF_INVERT_POWER, default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_unit_id(config[CONF_UNIT_ID]))
    cg.add(var.set_phase_config(config[CONF_PHASES]))
    cg.add(var.set_serial(config[CONF_SERIAL]))
    cg.add(var.set_invert_power(config[CONF_INVERT_POWER]))

    cg.add(var.set_power_l1_sensor(await cg.get_variable(config[CONF_POWER_L1])))
    cg.add(var.set_power_l2_sensor(await cg.get_variable(config[CONF_POWER_L2])))
    cg.add(var.set_power_l3_sensor(await cg.get_variable(config[CONF_POWER_L3])))
    cg.add(var.set_import_sensor(await cg.get_variable(config[CONF_IMPORT_ENERGY])))
    cg.add(var.set_export_sensor(await cg.get_variable(config[CONF_EXPORT_ENERGY])))

    if CONF_VOLTAGE_L1 in config:
        cg.add(var.set_voltage_l1_sensor(await cg.get_variable(config[CONF_VOLTAGE_L1])))
    if CONF_VOLTAGE_L2 in config:
        cg.add(var.set_voltage_l2_sensor(await cg.get_variable(config[CONF_VOLTAGE_L2])))
    if CONF_VOLTAGE_L3 in config:
        cg.add(var.set_voltage_l3_sensor(await cg.get_variable(config[CONF_VOLTAGE_L3])))
    if CONF_FREQUENCY in config:
        cg.add(var.set_frequency_sensor(await cg.get_variable(config[CONF_FREQUENCY])))
