"""wheel_loader.fbx をリンク単位の STL に分割出力する Blender スクリプト。

実行方法:
    blender --background --python export_link_meshes.py

処理内容:
    1. FBX をインポートし、全体を Z 軸まわりに +90 度回転して
       ROS 慣習（+X 前方, +Y 左, +Z 上）に合わせる
    2. リンクごとに visual / collision メッシュを結合し、
       joint ピボットを原点として STL を出力する
    3. 各リンクのピボット座標（ワールド系）を link_pivots.json に書き出す
"""
import json
import math
import os

import bpy
from mathutils import Matrix, Vector

REPOSITORY_ROOT = "/Users/nyanziba/work/OpenConstructionSimulator"
FBX_PATH = os.path.join(
    REPOSITORY_ROOT, "Assets/OcsVehicle/Models/wheel_loader.fbx"
)
OUTPUT_ROOT = os.path.join(REPOSITORY_ROOT, "urdf/wheel_loader")
MESH_OUTPUT_DIRECTORY = os.path.join(OUTPUT_ROOT, "meshes")
PIVOT_JSON_PATH = os.path.join(OUTPUT_ROOT, "link_pivots.json")

# リンク定義: リンク名 -> (ピボット元オブジェクト名, visualメッシュ接頭辞リスト,
#                          collisionメッシュ接頭辞リスト)
LINK_DEFINITIONS = {
    "base_link": {
        "pivot_object": "Body",
        "visual_prefixes": ["Body", "Handle"],
        "collision_prefixes": ["Body.Collision"],
    },
    "front_frame": {
        "pivot_object": "SubBody",
        "visual_prefixes": ["SubBody"],
        "collision_prefixes": ["SubBody.Collision"],
    },
    "boom": {
        "pivot_object": "Boom",
        "visual_prefixes": ["Boom"],
        "collision_prefixes": ["Boom.Collider"],
    },
    "bucket": {
        "pivot_object": "Bucket",
        "visual_prefixes": ["Bucket"],
        "collision_prefixes": ["Bucket.Collision"],
    },
    "bucket_link": {
        "pivot_object": "BucketLink",
        "visual_prefixes": ["BucketLink"],
        "collision_prefixes": ["BucketLink.Collider"],
    },
    "boom_cylinder": {
        "pivot_object": "BoomCylinder",
        "visual_prefixes": ["BoomCylinder"],
        "collision_prefixes": ["BoomCylinder.Collider"],
    },
    "boom_cylinder_rod": {
        "pivot_object": "BoomCylinderRod",
        "visual_prefixes": ["BoomCylinderRod"],
        "collision_prefixes": [],
    },
    "bucket_cylinder": {
        "pivot_object": "BucketCylinder",
        "visual_prefixes": ["BucketCylinder"],
        "collision_prefixes": ["BucketCylinder.Collider"],
    },
    "bucket_cylinder_rod": {
        "pivot_object": "BucketCylinderRod",
        "visual_prefixes": ["BucketCylinderRod"],
        "collision_prefixes": [],
    },
    "wheel_front_left": {
        "pivot_object": "Wheel.FL",
        "visual_prefixes": ["Wheel.FL"],
        "collision_prefixes": [],
    },
    "wheel_front_right": {
        "pivot_object": "Wheel.FR",
        "visual_prefixes": ["Wheel.FR"],
        "collision_prefixes": [],
    },
    "wheel_rear_left": {
        "pivot_object": "Wheel.RL",
        "visual_prefixes": ["Wheel.RL"],
        "collision_prefixes": [],
    },
    "wheel_rear_right": {
        "pivot_object": "Wheel.RR",
        "visual_prefixes": ["Wheel.RR"],
        "collision_prefixes": [],
    },
}


def prefix_matches(object_name, prefixes, excluded_names):
    """接頭辞一致でメッシュを選ぶ。他リンクに属する名前は除外する。"""
    for prefix in prefixes:
        if object_name == prefix or object_name.startswith(prefix + "."):
            if object_name in excluded_names:
                continue
            return True
    return False


def collect_mesh_names(prefixes, kind):
    """kind: 'visual' なら Collision/Collider を除外、'collision' ならそれのみ。"""
    selected_names = []
    for scene_object in bpy.context.scene.objects:
        if scene_object.type != "MESH":
            continue
        name = scene_object.name
        is_collision_mesh = ".Collision" in name or ".Collider" in name
        if kind == "visual" and is_collision_mesh:
            continue
        if kind == "collision" and not is_collision_mesh:
            continue
        selected_names.append(name)
    matched_names = []
    for name in selected_names:
        for prefix in prefixes:
            if name == prefix or name.startswith(prefix + "."):
                matched_names.append(name)
                break
    return matched_names


def resolve_visual_conflicts(link_definitions):
    """接頭辞の包含関係（Boom と BoomCylinder 等）による重複割当を解消する。

    より長い接頭辞に一致した名前を優先し、短い接頭辞側から除外する。
    """
    assignments = {}
    for link_name, definition in link_definitions.items():
        for kind in ("visual", "collision"):
            prefixes = definition[
                "visual_prefixes" if kind == "visual" else "collision_prefixes"
            ]
            for mesh_name in collect_mesh_names(prefixes, kind):
                best_prefix = max(
                    (p for p in prefixes
                     if mesh_name == p or mesh_name.startswith(p + ".")),
                    key=len,
                )
                key = (kind, mesh_name)
                current = assignments.get(key)
                if current is None or len(best_prefix) > current[1]:
                    assignments[key] = (link_name, len(best_prefix))
    resolved = {
        link_name: {"visual": [], "collision": []}
        for link_name in link_definitions
    }
    for (kind, mesh_name), (link_name, _) in assignments.items():
        resolved[link_name][kind].append(mesh_name)
    return resolved


def export_mesh_group(mesh_names, pivot_world, output_path):
    """メッシュ群を複製・結合し、pivot_world を原点として出力する。

    拡張子が .stl なら STL、.dae なら Collada（マテリアル色付き）で出力する。
    """
    bpy.ops.object.select_all(action="DESELECT")
    duplicated_objects = []
    for mesh_name in mesh_names:
        source_object = bpy.data.objects[mesh_name]
        duplicate = source_object.copy()
        duplicate.data = source_object.data.copy()
        bpy.context.collection.objects.link(duplicate)
        # Armature 等の親子関係を外し、ワールド変換を明示的に引き継ぐ
        # （親付きのままだと transform_apply が親オフセットを焼き込めない）
        world_matrix = source_object.matrix_world.copy()
        duplicate.parent = None
        duplicate.matrix_world = world_matrix
        duplicated_objects.append(duplicate)

    for duplicate in duplicated_objects:
        duplicate.select_set(True)
    bpy.context.view_layer.objects.active = duplicated_objects[0]
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if len(duplicated_objects) > 1:
        bpy.ops.object.join()
    joined_object = bpy.context.view_layer.objects.active

    # ピボットを原点へ移動してローカル座標系を joint 基準にする
    joined_object.location = joined_object.location - Vector(pivot_world)
    bpy.ops.object.transform_apply(location=True)

    bpy.ops.object.select_all(action="DESELECT")
    joined_object.select_set(True)
    if output_path.endswith(".dae"):
        bpy.ops.wm.collada_export(
            filepath=output_path,
            selected=True,
            apply_modifiers=True,
            triangulate=True,
        )
    else:
        bpy.ops.wm.stl_export(
            filepath=output_path,
            export_selected_objects=True,
            global_scale=1.0,
            apply_modifiers=True,
        )
    bpy.data.objects.remove(joined_object, do_unlink=True)


def main():
    os.makedirs(MESH_OUTPUT_DIRECTORY, exist_ok=True)

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=FBX_PATH)

    # ROS 慣習に合わせて全ルートを Z 軸まわりに +90 度回転（-Y 前方 -> +X 前方）
    rotation_z90 = Matrix.Rotation(math.radians(90.0), 4, "Z")
    for scene_object in bpy.context.scene.objects:
        if scene_object.parent is None:
            scene_object.matrix_world = rotation_z90 @ scene_object.matrix_world
    bpy.context.view_layer.update()

    resolved_meshes = resolve_visual_conflicts(LINK_DEFINITIONS)

    pivot_records = {}
    for link_name, definition in LINK_DEFINITIONS.items():
        pivot_object = bpy.data.objects[definition["pivot_object"]]
        pivot_world = list(pivot_object.matrix_world.to_translation())
        pivot_records[link_name] = [round(value, 6) for value in pivot_world]

        visual_names = resolved_meshes[link_name]["visual"]
        collision_names = resolved_meshes[link_name]["collision"]
        if not visual_names:
            raise RuntimeError(f"visual mesh not found for link: {link_name}")

        export_mesh_group(
            visual_names,
            pivot_world,
            os.path.join(MESH_OUTPUT_DIRECTORY, f"{link_name}_visual.dae"),
        )
        # collision メッシュが無いリンクの代替 collision 用に STL も併せて出力
        export_mesh_group(
            visual_names,
            pivot_world,
            os.path.join(MESH_OUTPUT_DIRECTORY, f"{link_name}_visual.stl"),
        )
        if collision_names:
            export_mesh_group(
                collision_names,
                pivot_world,
                os.path.join(
                    MESH_OUTPUT_DIRECTORY, f"{link_name}_collision.stl"
                ),
            )
        print(f"[export] {link_name}: visual={visual_names} "
              f"collision={collision_names} pivot={pivot_records[link_name]}")

    with open(PIVOT_JSON_PATH, "w", encoding="utf-8") as json_file:
        json.dump(pivot_records, json_file, indent=2)
    print(f"[export] pivots written to {PIVOT_JSON_PATH}")


main()
