from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sim = RemoteAPIClient().require("sim")
tipos = {sim.sceneobject_shape: "shape", sim.sceneobject_joint: "joint",
         sim.sceneobject_dummy: "dummy", sim.sceneobject_proximitysensor: "prox_sensor",
         sim.sceneobject_visionsensor: "vision_sensor"}
for h in sim.getObjectsInTree(sim.handle_scene):
    print(f"{sim.getObjectAlias(h, 1):<45} {tipos.get(sim.getObjectType(h), 'outro')}")
