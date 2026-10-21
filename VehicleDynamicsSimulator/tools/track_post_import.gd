# Post-import for converted Assetto Corsa track visuals (tools/asset_pipeline/kn5_convert.py).
# - Visual only: removes any collision/physics node the importer could have produced.
# - Assigns the generated StandardMaterial3D (<track>/materials/<name>.tres) to every surface by material name.
# - Applies render hints encoded in node names: "__ns" = no shadow casting (transparent, foliage, fences, minor
#   objects), "__vr<m>" = visibility range end in metres (minor objects disappear in the distance).
@tool
extends EditorScenePostImport

var _material_dir := ""
var _stats := {"meshes": 0, "no_shadow": 0, "ranged": 0, "materials": 0, "missing_materials": 0, "removed_physics": 0}

func _post_import(scene: Node) -> Object:
	_material_dir = get_source_file().get_base_dir() + "/materials/"
	_walk(scene)
	print("track post-import ", get_source_file(), ": ", _stats)
	return scene

func _walk(node: Node) -> void:
	for child in node.get_children():
		if child is CollisionObject3D or child is CollisionShape3D:
			child.get_parent().remove_child(child)
			child.free()
			_stats.removed_physics += 1
			continue
		_walk(child)
	if node is MeshInstance3D or node is ImporterMeshInstance3D:
		_process_mesh(node)

func _process_mesh(node: Node3D) -> void:
	_stats.meshes += 1
	var name := String(node.name)
	if name.contains("__ns"):
		node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		_stats.no_shadow += 1
	var at := name.find("__vr")
	if at >= 0:
		node.visibility_range_end = float(name.substr(at + 4).get_slice("_", 0))
		node.visibility_range_end_margin = 25.0
		node.visibility_range_fade_mode = GeometryInstance3D.VISIBILITY_RANGE_FADE_SELF
		_stats.ranged += 1
	var mesh = node.mesh
	if mesh == null:
		return
	for i in mesh.get_surface_count():
		var current = mesh.get_surface_material(i) if node is ImporterMeshInstance3D else mesh.surface_get_material(i)
		var material_name: String = current.resource_name if current != null else ""
		var path := _material_dir + material_name + ".tres"
		if material_name != "" and ResourceLoader.exists(path):
			var mat := load(path)
			if node is ImporterMeshInstance3D:
				mesh.set_surface_material(i, mat)
			else:
				mesh.surface_set_material(i, mat)
			_stats.materials += 1
		else:
			_stats.missing_materials += 1
