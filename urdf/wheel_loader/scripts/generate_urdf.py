"""link_pivots.json から wheel_loader.urdf を生成するスクリプト。

実行方法:
    python3 generate_urdf.py

座標系は ROS 慣習（+X 前方, +Y 左, +Z 上, m 単位）。
機体姿勢は world -> yaw -> pitch -> roll -> base_link の直列受動 joint で表現する。
油圧シリンダーは閉リンク機構のため URDF では fixed joint として表現する。
"""
import json
import math
import os

SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
OUTPUT_ROOT = os.path.dirname(SCRIPT_DIRECTORY)
PIVOT_JSON_PATH = os.path.join(OUTPUT_ROOT, "link_pivots.json")
URDF_OUTPUT_PATH = os.path.join(OUTPUT_ROOT, "wheel_loader.urdf")

# OEDO-Webrtc-Frontend のアップロード処理は package://<フォルダ名>/ 形式の
# URL のみ Blob URL に置換するため、この形式で参照する
MESH_PATH_PREFIX = "package://wheel_loader/meshes"

WHEEL_RADIUS = 2.125
WHEEL_WIDTH = 1.5

# リンク質量の概算値 [kg]（実機仕様が不明なため寸法からの推定値）
LINK_MASSES = {
    "base_link": 8000.0,
    "front_frame": 4000.0,
    "boom": 1200.0,
    "bucket": 900.0,
    "bucket_link": 300.0,
    "boom_cylinder": 120.0,
    "boom_cylinder_rod": 60.0,
    "bucket_cylinder": 120.0,
    "bucket_cylinder_rod": 60.0,
    "wheel_front_left": 350.0,
    "wheel_front_right": 350.0,
    "wheel_rear_left": 350.0,
    "wheel_rear_right": 350.0,
}

# 慣性計算用の概算バウンディングボックス寸法 [m] (x, y, z)
LINK_BOUNDING_BOXES = {
    "base_link": (8.6, 7.1, 5.7),
    "front_frame": (6.2, 6.6, 5.6),
    "boom": (8.3, 3.6, 1.7),
    "bucket": (3.4, 7.9, 3.6),
    "bucket_link": (3.7, 4.1, 4.5),
    "boom_cylinder": (2.9, 3.9, 0.6),
    "boom_cylinder_rod": (2.4, 4.1, 0.5),
    "bucket_cylinder": (3.6, 4.0, 0.6),
    "bucket_cylinder_rod": (0.7, 3.6, 4.0),
}


def format_vector(vector):
    return " ".join(f"{value:.6f}" for value in vector)


def subtract(child_pivot, parent_pivot):
    return [child - parent for child, parent in zip(child_pivot, parent_pivot)]


def box_inertia_xml(mass, dimensions):
    size_x, size_y, size_z = dimensions
    ixx = mass / 12.0 * (size_y**2 + size_z**2)
    iyy = mass / 12.0 * (size_x**2 + size_z**2)
    izz = mass / 12.0 * (size_x**2 + size_y**2)
    return (
        f'      <inertia ixx="{ixx:.3f}" ixy="0" ixz="0" '
        f'iyy="{iyy:.3f}" iyz="0" izz="{izz:.3f}"/>'
    )


def cylinder_inertia_xml(mass, radius, length):
    ixx = mass / 12.0 * (3.0 * radius**2 + length**2)
    izz = 0.5 * mass * radius**2
    return (
        f'      <inertia ixx="{ixx:.3f}" ixy="0" ixz="0" '
        f'iyy="{ixx:.3f}" iyz="0" izz="{izz:.3f}"/>'
    )


def link_xml(link_name, has_collision_mesh, is_wheel):
    mass = LINK_MASSES[link_name]
    lines = [f'  <link name="{link_name}">']

    lines.append("    <inertial>")
    lines.append(f'      <mass value="{mass}"/>')
    lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
    if is_wheel:
        # 車輪はローカル Y 軸が回転軸のため rpy で合わせた円柱慣性を近似使用
        lines.append(cylinder_inertia_xml(mass, WHEEL_RADIUS, WHEEL_WIDTH))
    else:
        lines.append(box_inertia_xml(mass, LINK_BOUNDING_BOXES[link_name]))
    lines.append("    </inertial>")

    lines.append("    <visual>")
    lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
    lines.append("      <geometry>")
    lines.append(
        f'        <mesh filename="{MESH_PATH_PREFIX}/{link_name}_visual.dae"/>'
    )
    lines.append("      </geometry>")
    lines.append("    </visual>")

    lines.append("    <collision>")
    if is_wheel:
        half_pi = math.pi / 2.0
        lines.append(f'      <origin xyz="0 0 0" rpy="{half_pi:.6f} 0 0"/>')
        lines.append("      <geometry>")
        lines.append(
            f'        <cylinder radius="{WHEEL_RADIUS}" '
            f'length="{WHEEL_WIDTH}"/>'
        )
        lines.append("      </geometry>")
    else:
        mesh_kind = "collision" if has_collision_mesh else "visual"
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append("      <geometry>")
        lines.append(
            f'        <mesh filename="{MESH_PATH_PREFIX}/'
            f'{link_name}_{mesh_kind}.stl"/>'
        )
        lines.append("      </geometry>")
    lines.append("    </collision>")
    lines.append("  </link>")
    return "\n".join(lines)


def dummy_link_xml(link_name):
    return (
        f'  <link name="{link_name}">\n'
        "    <inertial>\n"
        '      <mass value="0.001"/>\n'
        '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        '      <inertia ixx="0.0001" ixy="0" ixz="0" '
        'iyy="0.0001" iyz="0" izz="0.0001"/>\n'
        "    </inertial>\n"
        "  </link>"
    )


def joint_xml(joint_name, joint_type, parent, child, origin_xyz,
              axis=None, lower=None, upper=None,
              effort=100000.0, velocity=2.0):
    lines = [f'  <joint name="{joint_name}" type="{joint_type}">']
    lines.append(f'    <origin xyz="{format_vector(origin_xyz)}" rpy="0 0 0"/>')
    lines.append(f'    <parent link="{parent}"/>')
    lines.append(f'    <child link="{child}"/>')
    if axis is not None:
        lines.append(f'    <axis xyz="{format_vector(axis)}"/>')
    if joint_type == "revolute":
        lines.append(
            f'    <limit lower="{lower}" upper="{upper}" '
            f'effort="{effort}" velocity="{velocity}"/>'
        )
    elif joint_type == "continuous":
        lines.append(f'    <limit effort="{effort}" velocity="{velocity}"/>')
    lines.append("  </joint>")
    return "\n".join(lines)


def main():
    with open(PIVOT_JSON_PATH, encoding="utf-8") as json_file:
        pivots = json.load(json_file)

    mesh_directory = os.path.join(OUTPUT_ROOT, "meshes")
    sections = ['<?xml version="1.0"?>', '<robot name="wheel_loader">']

    # ---- 姿勢表現用の受動 joint 列: world -> yaw -> pitch -> roll -> base ----
    sections.append(dummy_link_xml("world"))
    sections.append(dummy_link_xml("attitude_yaw_link"))
    sections.append(dummy_link_xml("attitude_pitch_link"))
    pi = math.pi
    sections.append(joint_xml(
        "attitude_yaw", "revolute", "world", "attitude_yaw_link",
        [0.0, 0.0, 0.0], axis=[0.0, 0.0, 1.0], lower=-pi, upper=pi,
    ))
    sections.append(joint_xml(
        "attitude_pitch", "revolute", "attitude_yaw_link",
        "attitude_pitch_link",
        [0.0, 0.0, 0.0], axis=[0.0, 1.0, 0.0], lower=-pi / 2, upper=pi / 2,
    ))
    sections.append(joint_xml(
        "attitude_roll", "revolute", "attitude_pitch_link", "base_link",
        [0.0, 0.0, 0.0], axis=[1.0, 0.0, 0.0], lower=-pi / 2, upper=pi / 2,
    ))

    # ---- 実体リンク ----
    for link_name in LINK_MASSES:
        has_collision_mesh = os.path.exists(
            os.path.join(mesh_directory, f"{link_name}_collision.stl")
        )
        is_wheel = link_name.startswith("wheel_")
        sections.append(link_xml(link_name, has_collision_mesh, is_wheel))

    # ---- 駆動系 joint ----
    # 中折れ操舵（アーティキュレーション）: 実機の一般的な操舵角 ±40 度
    articulation_limit = math.radians(40.0)
    sections.append(joint_xml(
        "articulation", "revolute", "base_link", "front_frame",
        subtract(pivots["front_frame"], pivots["base_link"]),
        axis=[0.0, 0.0, 1.0],
        lower=-articulation_limit, upper=articulation_limit,
    ))
    sections.append(joint_xml(
        "boom_pivot", "revolute", "front_frame", "boom",
        subtract(pivots["boom"], pivots["front_frame"]),
        axis=[0.0, 1.0, 0.0], lower=-0.6, upper=0.9,
    ))
    # バケット取付部のチルトローテーター:
    #   bucket_pivot(上下カール, Y軸) -> tiltrotator_tilt(左右チルト, X軸 ±45度)
    #   -> tiltrotator_rotate(連続回転, Z軸) -> bucket
    sections.append(dummy_link_xml("tiltrotator_mount"))
    sections.append(dummy_link_xml("tiltrotator_rotator"))
    sections.append(joint_xml(
        "bucket_pivot", "revolute", "boom", "tiltrotator_mount",
        subtract(pivots["bucket"], pivots["boom"]),
        axis=[0.0, 1.0, 0.0], lower=-1.2, upper=0.8,
    ))
    tilt_limit = math.radians(45.0)
    sections.append(joint_xml(
        "tiltrotator_tilt", "revolute", "tiltrotator_mount",
        "tiltrotator_rotator",
        [0.0, 0.0, 0.0], axis=[1.0, 0.0, 0.0],
        lower=-tilt_limit, upper=tilt_limit,
    ))
    sections.append(joint_xml(
        "tiltrotator_rotate", "continuous", "tiltrotator_rotator", "bucket",
        [0.0, 0.0, 0.0], axis=[0.0, 0.0, 1.0],
    ))
    sections.append(joint_xml(
        "bucket_link_pivot", "revolute", "boom", "bucket_link",
        subtract(pivots["bucket_link"], pivots["boom"]),
        axis=[0.0, 1.0, 0.0], lower=-1.2, upper=1.2,
    ))

    # ---- 油圧シリンダー（閉リンクのため fixed で近似）----
    fixed_cylinder_joints = [
        ("boom_cylinder_mount", "front_frame", "boom_cylinder"),
        ("boom_cylinder_rod_mount", "boom", "boom_cylinder_rod"),
        ("bucket_cylinder_mount", "boom", "bucket_cylinder"),
        ("bucket_cylinder_rod_mount", "bucket_link", "bucket_cylinder_rod"),
    ]
    for joint_name, parent, child in fixed_cylinder_joints:
        sections.append(joint_xml(
            joint_name, "fixed", parent, child,
            subtract(pivots[child], pivots[parent]),
        ))

    # ---- 車輪 joint ----
    wheel_parents = {
        "wheel_front_left": "front_frame",
        "wheel_front_right": "front_frame",
        "wheel_rear_left": "base_link",
        "wheel_rear_right": "base_link",
    }
    for wheel_name, parent in wheel_parents.items():
        sections.append(joint_xml(
            f"{wheel_name}_axle", "continuous", parent, wheel_name,
            subtract(pivots[wheel_name], pivots[parent]),
            axis=[0.0, 1.0, 0.0],
        ))

    sections.append("</robot>")

    with open(URDF_OUTPUT_PATH, "w", encoding="utf-8") as urdf_file:
        urdf_file.write("\n".join(sections) + "\n")
    print(f"URDF written to {URDF_OUTPUT_PATH}")


main()
