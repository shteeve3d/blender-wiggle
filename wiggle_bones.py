bl_info = {
    "name": "Wiggle Bone",
    "author": "Steve Miller",
    "version": (1, 3, 1),
    "blender": (2, 80, 0),
    "location": "Properties > Bone",
    "description": "Simulates simple jiggle physics on bones",
    "warning": "",
    "wiki_url": "",
    "category": "Animation",
}

import bpy, math, mathutils
from mathutils import Vector,Matrix,Euler,Quaternion
from bpy.app.handlers import persistent

skip = False 

#returns name of parent in jiggle list or None
def check_parent(b, list):
    if list.find(b.name) >= 0:
        return b.name
    else:
        if b.parent:
            return check_parent(b.parent, list)
        else:
            return None

def treeprint(forest,list):
    for tree in forest:
        print(tree['name'])
        item = list.add()
        item.name = tree['name']
        if 'children' in tree:
            treeprint(tree['children'],list)
            
                   
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
            b['loc_start'] = b.location.copy()
    
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
                    #print("added %s" %b.name)
            #sort bone list so parent bones are processed before child bones
            nodes = {}
            for item in ob.jiggle_list:
                nodes[item.name] = {'name':item.name}
            
            forest = []    
            for item in ob.jiggle_list:
                node = nodes[item.name]
                parent_name = None
                if ob.pose.bones[item.name].parent:
                    parent_name = check_parent(ob.pose.bones[item.name].parent, ob.jiggle_list)
                if parent_name: #item has a parent in the list
                    parent = nodes[parent_name]
                    if not 'children' in parent:
                        parent['children'] = []
                    children = parent['children']
                    children.append(node)
                else:
                    forest.append(node)
                    
            ob.jiggle_list.clear()
            treeprint(forest,ob.jiggle_list)
                        
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
    
def gravity_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_gravity= a.jiggle_gravity
    skip = False
    
def translation_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_translation= a.jiggle_translation
    skip = False
                    
#return m2 translation vector in m1 space
def relative_vector(m1,m2):
    mat = m2.inverted() @ m1
    vec = (mat.inverted().to_euler().to_matrix().to_4x4() @ Matrix.Translation(mat.translation)).translation
    return vec

def jiggle_bone(b):
    #translational movement between frames in bone's >>previous<< orientation space
    vec = relative_vector(Matrix(b['jiggle_mat']), b.id_data.matrix_world @ b.matrix) * -1
    vecy = vec.y
    vec.y = 0 #y translation shouldn't affect y rotation, but store it for scaling

    
    #translational vector without any previous jiggle (and y)
    t1 = Matrix(b['t1'])
    t2 = (b.id_data.matrix_world @ b.matrix)
    t = relative_vector(t1, t2)
    print(t)
    b['t1'] = t2

    #rotational input between frames
    rot1 = Quaternion(b['rot1'])
    rot2 = (b.id_data.matrix_world @ b.matrix).to_quaternion()
    delta1 = (rot2.to_matrix().to_4x4().inverted() @ rot1.to_matrix().to_4x4()).to_euler()
    deltarot = Vector((delta1.z,-delta1.y,-delta1.x))/4
    #print(delta1)
    b['rot1']=rot2
    
    #gravity force vector from current orientation (from previous frame)
    g = bpy.context.scene.gravity * .01 * b.jiggle_gravity
    gvec = relative_vector(Matrix(b['jiggle_mat']).to_quaternion().to_matrix().to_4x4(), Matrix.Translation(g))
    gvec.y = 0
    
    tension = Vector(b.jiggle_spring)+vec+deltarot
    b.jiggle_velocity = Vector(b.jiggle_velocity)*(1-b.jiggle_dampen)-tension*b.jiggle_stiffness + gvec*(1-b.jiggle_stiffness)
    b.jiggle_spring = tension+Vector(b.jiggle_velocity)
    
    tension2 = Vector(b.jiggle_spring2)+t
    b.jiggle_velocity2 = Vector(b.jiggle_velocity2)*(1-b.jiggle_dampen)-tension2*b.jiggle_stiffness
    b.jiggle_spring2 = tension2 + Vector(b.jiggle_velocity2)
    
    #first frame should not consider any previous frame
    if bpy.context.scene.frame_current == bpy.context.scene.frame_start:
        vec = Vector((0,0,0))
        vecy = 0
        deltarot = Vector((0,0,0))
        b.jiggle_velocity = Vector((0,0,0))
        b.jiggle_spring = Vector((0,0,0))
        tension = Vector((0,0,0))
        
        b.jiggle_velocity2 = Vector((0,0,0))
        b.jiggle_spring2 = Vector((0,0,0))
        tension2 = Vector((0,0,0))

    #rotation is set via matrix so it can be applied locally before animated orientation changes)
    eulerRot = Euler((math.radians(Vector(b.jiggle_spring).z*-b.jiggle_amplitude), math.radians(Vector(b.jiggle_spring).y*-b.jiggle_amplitude),math.radians(Vector(b.jiggle_spring).x*+b.jiggle_amplitude)))
    
    #translation matrix
    trans = Matrix.Translation(Vector(b.jiggle_spring2) * b.jiggle_translation)
    #print(trans.translation)
    
        
    #if this works, properly store array of current influence values to re-apply
    c_influence = []
    if b.constraints:
        for c in b.constraints:
            #store current value here
            c_influence.append(c.influence)
            c.influence = 0
        bpy.context.scene.update()    
        
    new_mat = b.matrix @ trans @ eulerRot.to_matrix().to_4x4()
    
    b.matrix = new_mat
    b.scale.y = b.scale.y*(1-vecy*b.jiggle_stretch)
    
    for i, c in enumerate(b.constraints):
        #restore current value here
        c.influence = c_influence[i]
        
    bpy.context.scene.update() #update to benefit bones down the heirarchy
    
    #this becomes the new previous frame matrix:
    b['jiggle_mat']=b.id_data.matrix_world @ b.matrix
    

@persistent
def jiggle_bone_pre(self):
    if bpy.context.scene.jiggle_enable:
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
                                if b.jiggle_translation != 0:
                                    try:
                                        b.location = b['loc_start']
                                    except:
                                        b['loc_start'] = b.location
                                try:
                                    test = b['rot1']
                                except:
                                    b['rot1'] = (b.id_data.matrix_world @ b.matrix).to_quaternion()
                                try:
                                    test = b['t1']
                                except:
                                    b['t1'] = (b.id_data.matrix_world @ b.matrix)

@persistent                
def jiggle_bone_post(self):
    if bpy.context.scene.jiggle_enable:
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
        layout.prop(b, 'jiggle_translation')
        layout.prop(b, 'jiggle_stretch')
        layout.prop(b, 'jiggle_gravity')
        
class JiggleScenePanel(bpy.types.Panel):
    bl_label = 'Wiggle Scene'
    bl_idname = 'OBJECT_PT_jiggle_scene_panel'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'scene'
    
    def draw(self,context):
        layout = self.layout
        layout.prop(context.scene, 'jiggle_enable')
        
class jiggle_bone_item(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()

def register():
    
    bpy.utils.register_class(jiggle_bone_item)
    bpy.utils.register_class(JiggleBonePanel)
    bpy.utils.register_class(JiggleScenePanel)
    
    bpy.types.PoseBone.jiggle_spring = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    bpy.types.PoseBone.jiggle_velocity = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    
    bpy.types.PoseBone.jiggle_spring2 = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    bpy.types.PoseBone.jiggle_velocity2 = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    
    bpy.types.Scene.jiggle_enable = bpy.props.BoolProperty(
        name = 'Enabled:',
        description = 'Global toggle for all jiggle bones',
        default = True
    )
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
        name = 'Amplitude Rotation:',
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
    bpy.types.PoseBone.jiggle_gravity = bpy.props.FloatProperty(
        name = 'Gravity:',
        description = 'strength of gravity force',
        default = 0.5,
        update = gravity_update
    )
    bpy.types.PoseBone.jiggle_translation = bpy.props.FloatProperty(
        name = 'Amplitude Translation:',
        description = 'strength of translation for disconnected bones',
        default = 0.5,
        update = translation_update
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

#1.3.1 CHANGELOG

#jiggle translation for detached bones.

#better handles bone constraints (previously, would additively re-apply constraints on every frame). seems to work well for 'child of' and 'copy rotation/location' with offsets. however overriding constraints like copy transform kill the jiggle effect


#TODO

#allow a jiggled position for disconnected child objects [DONE]
#unkeyed translational 'start pos' [DONE]

#bake just jiggle to animation layer keyframes

#y-stretch should jiggle
#y-stretch should squash and stretch

#simple collision solution?

#lower priority:
#enabled button into wiggle bone, scene panel title for more compact ui
#object level enabled, allowing more granularity
#cleaner code for property updates (one function for multiple properties)
#performance tweak: only do context.update() on bones that have children