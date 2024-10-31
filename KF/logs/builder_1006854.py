# 2024-10-31T11:48:15.955526
import vitis

client = vitis.create_client()
client.set_workspace(path="/home/mgianell/KF")

comp = client.get_component(name="KalmanFil")
comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION_DEBUG")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION")

comp.execute(operation="C_SIMULATION_DEBUG")

comp.execute(operation="C_SIMULATION")

vitis.dispose()

