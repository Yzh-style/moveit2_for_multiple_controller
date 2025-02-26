bl_info = {
    "name": "URDF Exporter",
    "blender": (3, 0, 0),
    "category": "Import-Export",
    "description": "Export 3D model to URDF format."
}

import bpy
import os
from mathutils import Vector

def calculate_inertia(mass, bounds):
    """Calculate the inertia tensor for a cuboid."""
    x, y, z = bounds
    Ixx = (1 / 12) * mass * (y**2 + z**2)
    Iyy = (1 / 12) * mass * (x**2 + z**2)
    Izz = (1 / 12) * mass * (x**2 + y**2)
    return Ixx, Iyy, Izz

def export_urdf(filepath):
    """Export the current Blender scene to a URDF file."""
    scene = bpy.context.scene
    robot_name = "my_robot"
    
    with open(filepath, 'w') as urdf_file:
        urdf_file.write(f"<robot name=\"{robot_name}\">\n")

        for obj in scene.objects:
            if obj.type == 'MESH':
                # Get object properties
                position = obj.location
                rotation = obj.rotation_euler
                dimensions = obj.dimensions
                mass = obj.get("mass", 1.0)  # Default mass to 1.0 if not specified
                
                # Calculate inertia
                Ixx, Iyy, Izz = calculate_inertia(mass, dimensions)
                
                # Write link and collision data
                urdf_file.write(f"  <link name=\"{obj.name}\">\n")
                urdf_file.write(f"    <visual>\n")
                urdf_file.write(f"      <origin xyz=\"{position.x} {position.y} {position.z}\" rpy=\"{rotation.x} {rotation.y} {rotation.z}\"/>\n")
                urdf_file.write(f"      <geometry>\n")
                urdf_file.write(f"        <box size=\"{dimensions.x} {dimensions.y} {dimensions.z}\"/>\n")
                urdf_file.write(f"      </geometry>\n")
                urdf_file.write(f"    </visual>\n")

                urdf_file.write(f"    <inertial>\n")
                urdf_file.write(f"      <mass value=\"{mass}\"/>\n")
                urdf_file.write(f"      <inertia ixx=\"{Ixx}\" ixy=\"0\" ixz=\"0\" iyy=\"{Iyy}\" iyz=\"0\" izz=\"{Izz}\"/>\n")
                urdf_file.write(f"    </inertial>\n")

                urdf_file.write(f"    <collision>\n")
                urdf_file.write(f"      <origin xyz=\"{position.x} {position.y} {position.z}\" rpy=\"{rotation.x} {rotation.y} {rotation.z}\"/>\n")
                urdf_file.write(f"      <geometry>\n")
                urdf_file.write(f"        <box size=\"{dimensions.x} {dimensions.y} {dimensions.z}\"/>\n")
                urdf_file.write(f"      </geometry>\n")
                urdf_file.write(f"    </collision>\n")

                urdf_file.write(f"  </link>\n")

        urdf_file.write("</robot>\n")

class ExportURDFOperator(bpy.types.Operator):
    """Export the current scene to URDF format"""
    bl_idname = "export_scene.urdf"
    bl_label = "Export URDF"

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        export_urdf(self.filepath)
        self.report({'INFO'}, f"URDF exported to {self.filepath}")
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

class URDFExporterPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "URDF Exporter"
    bl_idname = "OBJECT_PT_urdf_exporter"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'URDF'

    def draw(self, context):
        layout = self.layout
        layout.operator(ExportURDFOperator.bl_idname, text="Export URDF")

def menu_func_export(self, context):
    self.layout.operator(ExportURDFOperator.bl_idname, text="URDF Export (.urdf)")

def register():
    bpy.utils.register_class(ExportURDFOperator)
    bpy.utils.register_class(URDFExporterPanel)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)

def unregister():
    bpy.utils.unregister_class(ExportURDFOperator)
    bpy.utils.unregister_class(URDFExporterPanel)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)

if __name__ == "__main__":
    register()
