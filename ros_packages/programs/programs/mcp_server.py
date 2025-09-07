import time
import sys
import logging
from typing import Any, Dict, Optional, List, Tuple
from mcp.server.fastmcp import FastMCP
from datatypes.srv import ApplyJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pib_api_client import pose_client
import rclpy

# --- Logging ---
stdout_handler = logging.StreamHandler(sys.stdout)
stdout_handler.setLevel(logging.INFO)
stdout_handler.addFilter(lambda rec: rec.levelno <= logging.INFO)
stderr_handler = logging.StreamHandler()
stderr_handler.setLevel(logging.WARNING)
logging.basicConfig(
    level=logging.INFO,
    handlers=[stdout_handler, stderr_handler],
    format="[%(levelname)s] [%(asctime)s]: %(message)s",
    datefmt="%y-%m-%d %H:%M:%S", force=True
)

# --- Hilfsfunktionen (lokal, damit der MCP-Server autark ist) ---

def list_poses_internal() -> Tuple[bool, List[Dict[str, str]]]:
    ok, poses = pose_client.get_poses()
    if not ok:
        logging.error("getting poses failed.")
        return False, []
    return True, poses

def apply_pose_by_id(pose_id: str) -> bool:
    # ROS Node starten
    rclpy.init(args=None)
    node = rclpy.create_node("pose_executor")

    client = node.create_client(ApplyJointTrajectory, "apply_joint_trajectory")
    node.get_logger().info("waiting for 'apply_joint_trajectory' service...")
    client.wait_for_service()
    node.get_logger().info("service available")

    # Motorpositionen holen
    ok, motor_positions = pose_client.get_motor_positions_of_pose(pose_id)
    if not ok:
        node.get_logger().error("getting motor-positions of pose failed.")
        node.destroy_node()
        rclpy.shutdown()
        return False

    jt = JointTrajectory()
    jt.joint_names = []
    for mp in motor_positions["motorPositions"]:
        name = mp["motorName"]
        pos = mp["position"]
        if name not in jt.joint_names:
            jt.joint_names.append(name)
        pt = JointTrajectoryPoint()
        pt.positions.append(pos)
        jt.points.append(pt)

    req = ApplyJointTrajectory.Request()
    req.joint_trajectory = jt

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    resp: ApplyJointTrajectory.Response = future.result()
    success = bool(resp and resp.successful)
    if success:
        node.get_logger().info("pose successfully applied.")
    else:
        node.get_logger().error("applying pose failed.")

    node.destroy_node()
    rclpy.shutdown()
    return success

def apply_pose_by_name_internal(pose_name: str) -> bool:
    ok, poses = list_poses_internal()
    if not ok or not poses:
        logging.error("no poses available.")
        return False
    pose_id = next((p.get("id") for p in poses if p.get("name") == pose_name), None)
    if not pose_id:
        logging.error(f"Pose '{pose_name}' not found.")
        return False
    logging.info(f"Found pose '{pose_name}' -> {pose_id}")
    return apply_pose_by_id(pose_id)

# --- MCP-Server & Tools ---

mcp = FastMCP("pib MCP-Server", host="0.0.0.0", port=9696)

@mcp.tool(
    name="posen_liste",
    description="Gibt alle verfügbaren Roboterposen als Liste zurück."
)
def posen_liste() -> Dict[str, Any]:
    successful, data = list_poses_internal()
    if not successful:
        return {"poses": []}
    
    poses = data["poses"]
    names = [p["name"] for p in poses if "name" in p]
    return {"posen": names}

@mcp.tool(
    name="pose_ausfuehren",
    description="Führt eine Pose per Namen aus. Beispiel: pose_ausführen(name='Winken')"
)
def pose_ausfuehren(name: str) -> Dict[str, Any]:
    success = apply_pose_by_name_internal(name)
    return {"status": "ok" if success else "error", "pose": name}

def geste(name: str, args: Optional[Dict[str, Any]] = None):
    # result = run_gesture(name, **(args or {}))
    return {"status": "ok", "gesture": name, "result": "ging", "args": args or {}}

def main():
    try:
        mcp.run(transport="streamable-http")
    finally:
        time.sleep(0.1)

if __name__ == "__main__":
    main()
