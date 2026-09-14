import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import esp32_camera
from esphome.const import CONF_ID


CONF_CAMERA_ID = "camera_id"

geometrie_camera_app_ns = cg.esphome_ns.namespace("geometrie_camera_app")

GeometrieCameraApp = geometrie_camera_app_ns.class_(
    "GeometrieCameraApp",
    cg.Component,
)

DEPENDENCIES = ["web_server", "esp32_camera"]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GeometrieCameraApp),
        cv.Required(CONF_CAMERA_ID): cv.use_id(esp32_camera.ESP32Camera),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    camera = await cg.get_variable(config[CONF_CAMERA_ID])
    cg.add(var.set_camera(camera))
