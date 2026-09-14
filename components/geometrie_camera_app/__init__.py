import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.const import CONF_ID


geometrie_camera_app_ns = cg.esphome_ns.namespace("geometrie_camera_app")

GeometrieCameraApp = geometrie_camera_app_ns.class_(
    "GeometrieCameraApp",
    cg.Component,
)

# L'application enregistre ses routes API dans le serveur web ESPHome.
DEPENDENCIES = ["web_server"]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GeometrieCameraApp),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
