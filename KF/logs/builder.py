# 2024-10-31T15:06:19.025814
import vitis

client = vitis.create_client()
client.set_workspace(path="/home/mgianell/KF")

comp = client.get_component(name="KalmanFil")
comp.execute(operation="C_SIMULATION")

vitis.dispose()

