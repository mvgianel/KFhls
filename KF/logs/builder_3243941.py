# 2024-10-24T15:20:56.564318
import vitis

client = vitis.create_client()
client.set_workspace(path="/home/mgianell/KF")

comp = client.create_hls_component(name="KalmanFil", cfg_file = ["hls_config.cfg"], template = "empty_hls_component")

