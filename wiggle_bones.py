bl_info = {
    "name": "Wiggle Bone",
    "author": "Steve Miller",
    "version": (1, 2),
    "blender": (2, 80, 0),
    "location": "Properties > Bone",
    "description": "Simulates simple jiggle physics on bones",
    "warning": "",
    "wiki_url": "",
    "category": "Animation",
}

import bpy, math, mathutils
from mathutils import Vector,Matrix,Euler
from bpy.app.handlers import persistent

skip = False                
def jiggle_list_refresh_ui(self,context):
    global skip
    if (skip):
        return
    skip = True
    #apply to other selected pose bones
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_enable = a.jiggle_enable
            
    #store the current pose as rest pose for the selected bones (that toggled this refresh)        
    for b in bpy.context.selected_pose_bones:
        if b.jiggle_enable:
            if b.rotation_mode == 'QUATERNION':
                b['rot_start']=b.rotation_quaternion.copy().to_euler()
            else:
                b['rot_start']=b.rotation_euler.copy()
    
    #iterate through all objects and bones to construct jiggle lists
    bpy.context.scene.jiggle_list.clear()
    for ob in bpy.context.scene.objects:
        if ob.type == 'ARMATURE':
            ob.jiggle_list.clear()
            for b in ob.pose.bones:
                if b.jiggle_enable:
                    item=ob.jiggle_list.add()
                    item.name = b.name
                    b['jiggle_mat']=b.id_data.matrix_world @ b.matrix
                    print("added %s" %b.name)
            if ob.jiggle_list:
                item=bpy.context.scene.jiggle_list.add()
                item.name = ob.name
    skip = False
                
def stiffness_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_stiffness = a.jiggle_stiffness
    skip = False
          
def dampen_update(self,context):
    global skip
    if(skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_dampen = a.jiggle_dampen
    skip = False
            
def amplitude_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_amplitude = a.jiggle_amplitude
    skip = False
            
def stretch_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_stretch = a.jiggle_stretch
    skip = False
                    
#return m2 vector in m1 space
def relative_vector(m1,m2):
    mat = m2.inverted() @ m1
    vec = (mat.inverted().to_euler().to_matrix().to_4x4() @ Matrix.Translation(mat.translation)).translation
    return vec

def jiggle_bone(b):
    #translational movement between frames in bone's orientation space
    vec = relative_vector(b.id_data.matrix_world @ b.matrix, Matrix(b['jiggle_mat']))
    vecy = vec.y
    vec.y = 0 #y translation shouldn't affect y rotation, but store it for scaling

    #rotational movement between frames
    rot1 = b.id_data.convert_space(pose_bone = b, matrix=Matrix(b['jiggle_mat']),from_space='WORLD', to_space='LOCAL').to_euler()
    rot2 = b.id_data.convert_space(pose_bone = b, matrix=(b.id_data.matrix_world @ b.matrix),from_space='WORLD', to_space='LOCAL').to_euler()
    deltarot = Vector((rot1.z-rot2.z, rot2.y-rot1.y, rot2.x-rot1.x))
                        
    b['jiggle_mat']=b.id_data.matrix_world @ b.matrix
    tension = Vector(b.jiggle_spring)+vec+deltarot
    b.jiggle_velocity = (Vector(b.jiggle_velocity)-tension*b.jiggle_stiffness)*(1-b.jiggle_dampen)
    b.jiggle_spring = tension+Vector(b.jiggle_velocity)
    
    #first frame should not consider any previous frame
    if bpy.context.scene.frame_current == bpy.context.scene.frame_start:
        vec = Vector((0,0,0))
        vecy = 0
        deltarot = Vector((0,0,0))
        b.jiggle_velocity = Vector((0,0,0))
        b.jiggle_spring = Vector((0,0,0))
        tension = Vector((0,0,0))
    
    additional = Vector((0,0,0))
    if b.rotation_mode=='QUATERNION':
        additional = b.rotation_quaternion.to_euler()
    else:
        additional = b.rotation_euler
        
    if b.rotation_mode == 'QUATERNION':
        rotation_euler = b.rotation_quaternion.to_euler()
    else:
        rotation_euler = b.rotation_euler

    rotation_euler.x = additional.x + math.radians(tension.z*-b.jiggle_amplitude)
    rotation_euler.y = additional.y + math.radians(tension.y*-b.jiggle_amplitude)
    rotation_euler.z = additional.z + math.radians(tension.x*+b.jiggle_amplitude)
        
    if b.rotation_mode == 'QUATERNION':
        b.rotation_quaternion = rotation_euler.to_quaternion()
    else:
        b.rotation_euler = rotation_euler

    b.scale.y = 1-vecy*b.jiggle_stretch
    

@persistent
def jiggle_bone_pre(self):
    for item in bpy.context.scene.jiggle_list:
        if bpy.data.objects.find(item.name) >= 0:
            ob = bpy.data.objects[item.name]
            if ob.type == 'ARMATURE':
                for item2 in ob.jiggle_list:
                    if ob.pose.bones.find(item2.name) >= 0:
                        b = ob.pose.bones[item2.name]
                        if b.jiggle_enable:
                            b.scale.y = 1
                            if b.rotation_mode == 'QUATERNION':
                                try:
                                    b.rotation_quaternion = Euler(b['rot_start']).to_quaternion()
                                except:
                                    b['rot_start'] = b.rotation_quaternion.copy().to_euler()
                            else:
                                try:
                                    b.rotation_euler = Euler(b['rot_start'])
                                except:
                                    b['rot_start'] = b.rotation_euler.copy()

@persistent                
def jiggle_bone_post(self):
    for item in bpy.context.scene.jiggle_list:
        if bpy.data.objects.find(item.name) >= 0:
            ob = bpy.data.objects[item.name]
            if ob.type == 'ARMATURE':
                for item2 in ob.jiggle_list:
                    if ob.pose.bones.find(item2.name) >= 0:
                        b = ob.pose.bones[item2.name]
                        if b.jiggle_enable:
                            jiggle_bone(b)
                            #grab copy of matrix on late update of first frame (prevents freakouts on loop)
                            if bpy.context.scene.frame_current == bpy.context.scene.frame_start:
                                b['jiggle_mat']=b.id_data.matrix_world @ b.matrix
    
class JiggleBonePanel(bpy.types.Panel):
    bl_label = 'Wiggle Bone'
    bl_idname = 'OBJECT_PT_jiggle_panel'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'bone'
    
    @classmethod
    def poll(cls, context):
        return (context.object is not None and context.object.type == 'ARMATURE')
    
    def draw(self,context):
        layout = self.layout
        b = context.active_pose_bone
        layout.prop(b, 'jiggle_enable')
        layout.prop(b, 'jiggle_stiffness')
        layout.prop(b,'jiggle_dampen')
        layout.prop(b, 'jiggle_amplitude')
        layout.prop(b, 'jiggle_stretch')
        
class jiggle_bone_item(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()

def register():
    
    bpy.utils.register_class(jiggle_bone_item)
    bpy.utils.register_class(JiggleBonePanel)
    
    bpy.types.PoseBone.jiggle_spring = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    bpy.types.PoseBone.jiggle_velocity = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    bpy.types.Scene.jiggle_list = bpy.props.CollectionProperty(type=jiggle_bone_item)
    bpy.types.Object.jiggle_list = bpy.props.CollectionProperty(type=jiggle_bone_item)
    bpy.types.PoseBone.jiggle_enable = bpy.props.BoolProperty(
        name = 'Enabled:',
        description = 'activate as jiggle bone',
        default = False,
        update = jiggle_list_refresh_ui
    )
    bpy.types.PoseBone.jiggle_dampen = bpy.props.FloatProperty(
        name = 'Dampening:',
        description = '0-1 range of how much tension is lost per frame, higher values settle quicker',
        default = 0.2,
        update = dampen_update
    )
    bpy.types.PoseBone.jiggle_stiffness = bpy.props.FloatProperty(
        name = 'Stiffness:',
        description = '0-1 range of how quickly bone tries to get to neutral state, higher values give faster jiggle',
        default = 0.2,
        update = stiffness_update
    )
    bpy.types.PoseBone.jiggle_amplitude = bpy.props.FloatProperty(
        name = 'Amplitude:',
        description = 'Multiplier for the amplitude of the spring, higher values make larger jiggles',
        default = 30,
        update = amplitude_update
    )
    bpy.types.PoseBone.jiggle_stretch = bpy.props.FloatProperty(
        name = 'Stretching:',
        description = '0-1 range for how much the jiggle stretches the bone, higher values stretch more',
        default = 0.5,
        update = stretch_update
    )
    
#    bpy.app.handlers.frame_change_pre.clear()
#    bpy.app.handlers.frame_change_post.clear()
    bpy.app.handlers.frame_change_pre.append(jiggle_bone_pre)
    bpy.app.handlers.frame_change_post.append(jiggle_bone_post)

def unregister():
    bpy.utils.unregister_class(JiggleBonePanel)
    bpy.utils.unregister_class(jiggle_bone_item)
    
    bpy.app.handlers.frame_change_pre.remove(jiggle_bone_pre)
    bpy.app.handlers.frame_change_post.remove(jiggle_bone_post)

if __name__ == "__main__":
    register()

#1.1 CHANGELOG

#y-rotation jiggle (twist jiggle)

#simple y-stretch is working better 

#wasn't handling object level transforms correctly (object rotations would cause glitches, translates wouldn't jiggle)

#handling of animated bones automatic and transparent, no longer needs checkbox.

#non animated bones define their rest pose from their orientation when the enabled button is clicked. update the rest pose by re-toggling the enabled state.

#multibone property updates



#TODO

#gravity?

#y-stretch should jiggle
#y-stretch should squash and stretch

#wiggle momentum transfer?
#   work down bone chain with bpy.context.scene.update() between each

#simple collision solution?