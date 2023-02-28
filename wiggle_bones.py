bl_info = {
    "name": "Wiggle Bone",
    "author": "Steve Miller",
    "version": (1, 5, 0),
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
import json

skip = False 
render = False
curframe = None

######## NEW STUFF STARTS ############################################
#Consider replacing generic python object with an actual node that doesn't need to be converted to dict on each access:
#class Jiggle_Node(bpy.types.PropertyGroup):
#   name: bpy.props.StringProperty()
#   children: bpy.props.CollectionProperty(type=Jiggle_Node)
#   bones: bpy.props.CollectionProperty(type=Jiggle_Node)
#   type: bpy.props.StringProperty() - is this necessary?


def find_parent(item, nodes):
    if item.parent:
        if item.parent.name in nodes:
            return item.parent
        else:
            return find_parent(item.parent, nodes)
    else:
        return None 
                            
def generate_jiggle_tree_bones(ob):
    print('GENERATING BONES FOR: ' + ob.name)
    nodes = {}
    
    for b in ob.pose.bones:
        if b.jiggle_enable:
            nodes[b.name] = {'children':{}, 'type':'BONE'}
            b['jiggle_mat']=b.id_data.matrix_world @ b.matrix
    tree = {}
    for bone_node in nodes:
        parent = find_parent(ob.pose.bones[bone_node],nodes)
        if parent:
            nodes[parent.name]['children'][bone_node] = nodes[bone_node]
        else:
            tree[bone_node] = nodes[bone_node]
    #print(tree)
    return tree

def generate_jiggle_tree():             
    #iterate through all objects and construct jiggle collider master list
    print('REFRESH JIGGLE LIST')
    
    nodes = {}
    #iterate through objects
    for ob in bpy.context.scene.objects:
        if ob.type == 'ARMATURE' and ob.data.jiggle_enable:
            nodes[ob.name] = {'children':{},'type':'OBJECT','bones':generate_jiggle_tree_bones(ob)}
    #print(nodes)
            
    tree = {}
    for ob_node in nodes:
        parent = find_parent(bpy.data.objects[ob_node], nodes)
        if parent:
            nodes[parent.name]['children'][ob_node]=nodes[ob_node]
        else:
            tree[ob_node] =nodes[ob_node]
            
    bpy.context.scene['jiggle_tree'] = tree #json.dumps(tree)
    
##################### NEW STUFF ENDS ##################################
def update_tree(self,context):
    generate_jiggle_tree()
                       
def jiggle_list_refresh_ui(self,context):
    global skip
    if (skip):
        return
    skip = True
    #apply to other selected pose bones
    a = bpy.context.active_pose_bone
    if bpy.context.selected_pose_bones:
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
                b['scale_start'] = b.scale.copy()
            
    #apply to other selected colliders:
    a = bpy.context.active_object
    if a and a.type == 'EMPTY':
        for b in bpy.context.selected_objects:
            if not b == a and b.type == 'EMPTY':
                b.jiggle_collider_enable = a.jiggle_collider_enable
                
    #iterate through all objects and construct jiggle collider master list
    print('REFRESH LIST')
    generate_jiggle_tree()
#    bpy.context.scene.jiggle_collider_list.clear()
#    for ob in bpy.context.scene.objects:
#        if ob.type == 'EMPTY' and ob.jiggle_collider_enable:
#            item = bpy.context.scene.jiggle_collider_list.add()
#            item.name = ob.name
#    
#    #iterate through all objects and bones to construct jiggle lists
#    bpy.context.scene.jiggle_list.clear()
#    for ob in bpy.context.scene.objects:
#        if ob.type == 'ARMATURE' and ob.data.jiggle_enable:
#            ob.jiggle_list.clear()
#            for b in ob.pose.bones:
#                if b.jiggle_enable:
#                    item=ob.jiggle_list.add()
#                    item.name = b.name
#                    b['jiggle_mat']=b.id_data.matrix_world @ b.matrix
#                    
#                    #add colliders
#                    b.jiggle_collider_list.clear()
#                    for c in bpy.context.scene.jiggle_collider_list:
#                        c = bpy.context.scene.objects[c.name]
#                        c_item=b.jiggle_collider_list.add()
#                        c_item.name = c.name
#                        c_item.theta_last = 0
#                        c_item.dir_last = 0
#                    #print("added %s" %b.name)
#            #sort bone list so parent bones are processed before child bones
#            nodes = {}
#            for item in ob.jiggle_list:
#                nodes[item.name] = {'name':item.name}
#            
#            forest = []    
#            for item in ob.jiggle_list:
#                node = nodes[item.name]
#                parent_name = None
#                if ob.pose.bones[item.name].parent:
#                    parent_name = check_parent(ob.pose.bones[item.name].parent, ob.jiggle_list)
#                if parent_name: #item has a parent in the list
#                    parent = nodes[parent_name]
#                    if not 'children' in parent:
#                        parent['children'] = []
#                    children = parent['children']
#                    children.append(node)
#                else:
#                    forest.append(node)
#                    
#            ob.jiggle_list.clear()
#            treeprint(forest,ob.jiggle_list)
#                        
#            if ob.jiggle_list:
#                item=bpy.context.scene.jiggle_list.add()
#                item.name = ob.name
    skip = False

def active_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_active = a.jiggle_active
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
    
def collision_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_collision= a.jiggle_collision
    skip = False
    
def margin_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_collision_margin= a.jiggle_collision_margin
    skip = False
    
def friction_update(self,context):
    global skip
    if (skip):
        return
    skip = True
    a = bpy.context.active_pose_bone
    for b in bpy.context.selected_pose_bones:
        if not b == a:
            b.jiggle_collision_friction= a.jiggle_collision_friction
    skip = False
                    
#return m2 translation vector in m1 space
def relative_vector(m1,m2):
    mat = m2.inverted() @ m1
    vec = (mat.inverted().to_euler().to_matrix().to_4x4() @ Matrix.Translation(mat.translation)).translation
    return vec

#def collide_bone(c_item, b, eulerRot):
#    print(b.name)
#    #possibly collision groups should be definable per object for localized performance
#    c = bpy.data.objects[c_item.name]
#    print('COLLIDER: ' + c.name)
#    
#    try :
#        c_last = Matrix(c['last'])
#    except:
#        c_last = c.matrix_world.copy()
#        c['last'] = c_last

#    r = c.empty_display_size * c.scale.x#display size includes world scaling
#    #this is incorrect, this size is empty size only and unchanging. somehow scaling is being accounted for elsewhere!
#    print('r:  ' + str(r))
#        
#    m_next = b.id_data.matrix_world @ b.matrix @ eulerRot.to_matrix().to_4x4()

#    #try to calculate vec of motion (both location and rotation of head?) in m_next space
#    m_last = Matrix(b['jiggle_mat']) 
#    
#    #alternate approach to better capture relative rotations
#    prev=Matrix.Translation(Matrix(c['last']).translation).inverted() @ m_last
#    nex = Matrix.Translation(c.matrix_world.translation).inverted() @ m_next
#    vec = relative_vector(nex @ Matrix.Translation((0,b.length,0)),prev @ Matrix.Translation((0,b.length,0)))
#    print('vec_2: ' + str(vec))
#    #vec = vec_2
#    
#    #now make a bone y rotation so local x points in the direction of motion
#    #????????? currently this doesn't handle division by zero case
#    theta_vec = math.atan(vec.z/vec.x)
#    if vec.x < 0:
#        if vec.z < 0:
#            theta_vec = theta_vec - math.radians(180)
#        else:
#            theta_vec = theta_vec + math.radians(180)
#    print('theta_vec: ' + str(math.degrees(-theta_vec)))
#    m_vec = m_next @ Euler((0,-theta_vec,0)).to_matrix().to_4x4()
#    
#    #get a circle slice of the sphere along that x axis, if it exists
#    d_sphere = relative_vector(m_vec, Matrix.Translation(c.matrix_world.translation))
#    print('d_sphere: ' + str(d_sphere))
#    d_slice = Vector((d_sphere.x,d_sphere.y)) #projection slice location

#    if abs(d_sphere.z) < r: #slice exists along radius
#        #r_slice = r * math.cos(d_sphere.z/r)
#        r_slice = math.sqrt(r*r-d_sphere.z*d_sphere.z)
#        print('r slice: ' + str(r_slice))
#        #d_sphere.z component can be used as an additional slippage rotation (when its zero its right on the centre and wouldn't slip)
#    else:
#        print('no slice')
#        r_slice = None
#          
#    #need to store a persistent dir_last so colliding direction is maintained 
#    try:
#        dir_last = c_item.dir_last
#    except:
#        c_item.dir_last = 0
#        dir_last = c_item.dir_last
#        
#    try:
#        theta_last = c_item.theta_last
#    except:
#        c_item.theta_last = theta_vec
#        theta_last = c_item.theta_last
#        
#    #print('theta v last: ' + str(math.degrees(b['theta_v_last'])))
#    print('theta last: ' + str(math.degrees(theta_last)))
#    #which way would next collide
#    dir_next = d_slice.x/abs(d_slice.x)
#    
#    
#    #try to find crossing
#    tip_next = (m_next @ Matrix.Translation((0,b.length,0))).to_translation()
#    tip_last = (m_last @ Matrix.Translation((0,b.length,0))).to_translation()
#   
#    #we want 3 values, P1,P2 line segment and P3 collision
#    #P2 should be relative change to collider (potentially animated) so its always at c_last position
#    P1 = tip_last
#    P2 = tip_next - (c.matrix_world-c_last).to_translation()
#    P3 = c_last.to_translation()
##    print(P1)
##    print(P2)
##    print(P3)
#    
#    #find 'u' distance along line P1,P2 that is closest to P3:
#    u = False
#    if (P2-P1).length:
#        #u = ((P3.x-P1.x)*(P2.x-P1.x)+(P3.y-P1.y)*(P2.y-P1.y)+(P3.z-P1.z)*(P2.z-P1.z))/(P2-P1).length
#        u = ((P3.x-P1.x)*(P2.x-P1.x)+(P3.y-P1.y)*(P2.y-P1.y)+(P3.z-P1.z)*(P2.z-P1.z))/((P2.x-P1.x)*(P2.x-P1.x)+(P2.y-P1.y)*(P2.y-P1.y)+(P2.z-P1.z)*(P2.z-P1.z))
#    
#    print('new cross test: ' + str(u))
#    
#    crossed = False
#    if 0 < u and u < 1:
#        #bone crosses sphere
#        #get P4 and u along P1P2 and see if its intersecting at its crossing
#        P4 = P1 + u*(P2-P1)
#        if abs((P4 - P3).length) < r:
#            crossed = True
#            print('new crossing intersection!!!!')
#    
#    #now we should do cases:
#    
#    if dir_last:
#        if crossed: #there was a crossing intersection between last and next
#            dir = dir_last
#            #try compare theta and theta last?
#            #if theta_vec * b['theta_v_last'] < 0: #they swapped
#            #if math.degrees(abs(theta_vec - b['theta_v_last'])) > 90:
#            if math.degrees(abs(theta_vec - theta_last)) > 90:
#                dir = -dir 
#                print('should i flip?')
#        else:
#            if (dir_last * dir_next) > 0: #direction has stayed the same
#                dir = dir_last
#            else: #direction switched but there was no crossing
#                dir = dir_next
#    else: #no previous frame
#        dir = dir_next
#      
#    c_item.theta_last = theta_vec   
#        
#    #store dir for next time
#    c_item.dir_last = dir
#    
#    print('dir %d' %dir)
#    
#    d = d_slice
#    r = r_slice
#    theta = theta_vec
#    
#    if r:
#        #scale radius with distance to have a margin that avoids b barely touching surface
#        margin = r*b.jiggle_collision_margin
#        factor = max(0,min((d.length-b.length)/(r+margin),1))
#        r = r + margin*factor
#        print('r: %f' %r)
#        
#        #could m_next collide?
#        if (d.length < b.length + r) and (d.length > r): #include no collision if bone is inside collider
#            #find collision in dir
#            
#            #tangent intersect:
#            #a lot of ifs could probably be made more elegant, but works for now
#            if (d.length*d.length < r*r + b.length*b.length):
#                print('along length')
#                r2 = math.asin(r/d.length)#always positive

#                #the collider is "in front" of the bone
#                if d.y < 0:
#                    if d.x < 0:
#                        print('below')
#                        if dir < 0:
#                            print('neg')
#                            r1= -math.atan(d.x/d.y)+r2
#                        else:
#                            print('pos')
#                            r1= math.radians(360)-math.atan(d.x/d.y)-r2
#                        #r1= r1+r2
#                    else:
#                        print('above')
#                        if dir > 0:
#                            print('pos')
#                            r1= -math.atan(d.x/d.y)-r2
#                        else:
#                            print('neg')
#                            if crossed:
#                                r1= -math.atan(d.x/d.y)+r2
#                            else:
#                                r1 = -math.atan(d.x/d.y)+r2-math.radians(360)
#                        #r1= r1-r2
#                        
#                #the collider is "behind" the bone
#                else:
#                    if d.x < 0:
#                        print('belo') #negative
#                        if dir < 0:
#                            print('neg')
#                            r1= -math.radians(180)-math.atan(d.x/d.y)+r2
#                        else: 
#                            print('pos')
#                            r1= math.radians(180)-math.atan(d.x/d.y)-r2

#                    else:
#                        print('abov') #positive
#                        if dir > 0:
#                            print('pos')
#                            r1= math.radians(180)-math.atan(d.x/d.y)-r2
#                        else:
#                            print('neg')
#                            if crossed:
#                                r1=-math.atan(d.x/d.y)+r2+math.radians(180)
#                            else:
#                                r1=-math.atan(d.x/d.y)+r2-math.radians(180)

#                #print(math.degrees(r1))          
#                
#            #tip intersect
#            else:
#                print('at tip')
#                #print(d)
#                #difference of squares goes here
#                r1 = -dir*math.acos((b.length*b.length + d.length*d.length - r*r)/(2*b.length*d.length)) - math.atan(d.x/d.y)
#                if d.y > 0:
#                    if crossed:
#                        r1 = r1 - math.radians(180)*dir
#                else:
#                    if not crossed:
#                        r1 = r1
#                #print(math.degrees(r1))
#            
#            print('r1: ' + str(math.degrees(r1)))
#            #if slip = 0, r1 stays the same
#            #if 0 < slip <= 1:
#                #if d_sphere.z = 0, r1 stays the same, there's no sideways direction to slip
#                #else as abs(d_sphere.z) approaches r, after r1 rotation, rotate on the spot to the tangent of the sphere, and unrotate by r1 * slip for it sliding out along the tangent
#            r_sphere = c.empty_display_size * c.scale.x
#            r1x = r1-((1-b.jiggle_collision_friction)*r1*d_sphere.z*d_sphere.z)/(r_sphere*r_sphere)
#            r1z = ((1-b.jiggle_collision_friction)*r1*d_sphere.z*r_slice)/(r_sphere*r_sphere)
#            
#            #verify this math works... can you just add the two rotations like this?
#            rx = r1x*math.cos(theta)
#            rz = r1x*math.sin(theta)
#            rx2 = r1z*math.sin(theta)
#            rz2 = r1z*math.cos(theta)
#            rx = rx + rx2
#            rz = rz - rz2
#            
#            
#            print('x: %f z: %f' %(math.degrees(rx), math.degrees(rz)))
#            
#            #return appropriate value
#            d_head = math.sqrt((-d.y-b.length)*(-d.y-b.length)+d.x*d.x)
#            if crossed:
#                print('crossing collision')
#                #return Euler((0,0,r1))
#                if b["r_max"]:
#                    if abs(r1) < abs(b["r_max"]):
#                        return None
#                b["c_largest"] = c.name
#                b["r_max"] = r1
#                return Euler((-rz,0,rx))

#                #rot_collision = Euler((-rz,0,rx))
#            elif ((d_head < r) or ((abs(d.x) < r) and (abs(d.y) < b.length) and (d.y < 0))):
#                print('collides')
#                #return Euler((0,0,r1))
#                if b["r_max"]:
#                    if abs(r1) < abs(b["r_max"]):
#                        return None
#                b["c_largest"] = c.name
#                b["r_max"] = r1
#                return Euler((-rz,0,rx))

#                #rot_collision = Euler((-rz,0,rx))
#            else:
#                print('no collision happens') #can collide but doesn't
#                #b['dir_last'] = dir_next
#                c_item.dir_last = dir_next
#                return None
#          
#        if (d.length < r):
#            print('base inside')
#        print('no collision possible')  
#        #b['dir_last'] = dir_next
#        c_item.dir_last = dir_next
#        return None #m_next can't collide
#    print('no r_slice')
#    return None

def reset_bone(b):
    #jiggle_bone_pre(b)
    #bpy.context.view_layer.update()
    b.jiggle_spring = b.jiggle_spring2 = b.jiggle_velocity = b.jiggle_velocity2 = Vector((0,0,0))
    #b['jiggle_mat']=b.id_data.matrix_world @ b.matrix
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)
    
    

######## NEW STUFF STARTS #######################################################################

def jiggle_bone_pre(b):
    #b.scale.y = Vector(b['scale_start']).y
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
            b['loc_start'] = b.location.copy()
    try:
        b.scale = b['scale_start']
    except:
            b['scale_start'] = b.scale.copy()
    try:
        test = b['rot1']
    except:
        b['rot1'] = (b.id_data.matrix_world @ b.matrix).to_quaternion()
    try:
        test = b['t1']
    except:
        b['t1'] = (b.id_data.matrix_world @ b.matrix)
    try:
        test = b['rot_col']
    except:
        b['rot_col'] = None
        
def jiggle_bone_post(b, new_b_mat):  
    
#    rate = bpy.context.scene.render.fps/bpy.context.scene.render.fps_base/24
    rate = bpy.context.scene.jiggle_rate
    
    #translational movement between frames in bone's >>previous<< orientation space
    vec = relative_vector(Matrix(b['jiggle_mat']), b.id_data.matrix_world @ new_b_mat) * -1
    vecy = vec.y
    vec.y = 0 #y translation shouldn't affect y rotation, but store it for scaling
    
    #translational vector without any previous jiggle (and y)
    t1 = Matrix(b['t1'])
    t2 = (b.id_data.matrix_world @ new_b_mat)
    t = relative_vector(t2, t1) #reversed so it is in the current frame's bone space?
    #ideally world space:
    t = t2.translation - t1.translation
    b['t1'] = t2

    #rotational input between frames
    rot1 = Quaternion(b['rot1'])
    #rot1 = Matrix(b['jiggle_mat']).to_quaternion()
    rot2 = (b.id_data.matrix_world @ b.matrix).to_quaternion()
    delta1 = (rot2.to_matrix().to_4x4().inverted() @ rot1.to_matrix().to_4x4()).to_euler()
    deltarot = Vector((delta1.z,-delta1.y,-delta1.x))/4
    #print(delta1)
    b['rot1']=rot2
    
    #gravity force vector from current orientation (from previous frame)
    g = bpy.context.scene.gravity * .01 * b.jiggle_gravity
    gvec = relative_vector(Matrix(b['jiggle_mat']).to_quaternion().to_matrix().to_4x4(), Matrix.Translation(g))
    #gvec = relative_vector(b.matrix.to_quaternion().to_matrix().to_4x4(), Matrix.Translation(g))
    #gvec.magnitude = g.magnitude
    #gvec.x = -0.01
    gvec.y = 0
    
    ##### NEW JIGGLE APPROACH ATTEMPT ########
    
#    d = Vector(b.jiggle_spring) + vec + deltarot + gvec
#    k = b.jiggle_stiffness
#    F = -k*d
#    new_d = d + F*b.jiggle_dampen
    
#    sum_of_forces = -(Vector(b.jiggle_spring)*b.jiggle_stiffness)+vec+deltarot+gvec
#    b.jiggle_velocity = Vector(b.jiggle_velocity)*(1-b.jiggle_dampen) + sum_of_forces
#    b.jiggle_spring = Vector(b.jiggle_spring) + Vector(b.jiggle_velocity)
        
    ######## FAILED!!!!! ###################
    
    #for rotational tension and jiggle
    #can i replace tension with just doing the jiggle spring? [yes]
    b.jiggle_spring = Vector(b.jiggle_spring)+vec+deltarot #input force
    b.jiggle_velocity = Vector(b.jiggle_velocity)*(1-b.jiggle_dampen)-Vector(b.jiggle_spring)*b.jiggle_stiffness + gvec*(1-b.jiggle_stiffness)
    b.jiggle_spring = Vector(b.jiggle_spring)+Vector(b.jiggle_velocity) / rate #physics forces if no collision
    
    
    #for translational tension and jiggle
    tension2 = Vector(b.jiggle_spring2)-t
    b.jiggle_velocity2 = Vector(b.jiggle_velocity2)*(1-b.jiggle_dampen)-tension2*b.jiggle_stiffness
    b.jiggle_spring2 = tension2 + Vector(b.jiggle_velocity2) / rate
    #can this all be calculated/stored variables in world space, and then converted to bone space?
    local_spring = t2.to_quaternion().to_matrix().to_4x4().inverted() @ Matrix.Translation(b.jiggle_spring2)
    
    #first frame or inactive should not consider any previous frame
    if ((bpy.context.scene.frame_current == bpy.context.scene.frame_start) and bpy.context.scene.jiggle_reset) or not b.jiggle_active:
        vec = Vector((0,0,0))
        vecy = 0
        deltarot = Vector((0,0,0))
        b.jiggle_velocity = Vector((0,0,0))
        b.jiggle_spring = Vector((0,0,0))
        tension = Vector((0,0,0))
        
        b.jiggle_velocity2 = Vector((0,0,0))
        b.jiggle_spring2 = Vector((0,0,0))
        tension2 = Vector((0,0,0))
        local_spring = Matrix.Identity(4)
        
        b['rot_col'] = Euler((0,0,0))
        b['dir_last'] = None
#        b['d_last'] = None

    #rotation is set via matrix so it can be applied locally before animated orientation changes)
    #this is rotation if there was no collision
    eulerRot = Euler((math.radians(Vector(b.jiggle_spring).z*-b.jiggle_amplitude*rate), math.radians(Vector(b.jiggle_spring).y*-b.jiggle_amplitude*rate),math.radians(Vector(b.jiggle_spring).x*+b.jiggle_amplitude*rate)))
    #translation matrix
    if not b.bone.use_connect:
        trans = Matrix.Translation(local_spring.translation * b.jiggle_translation)
    else:
        trans = Matrix.Identity(4)
    #print(trans.translation)    
    
#    #COLLISIONS!!!

#    b["r_max"] = None
#    rot_collision = None
#    b["c_largest"] = None
#    
#    #too many else's, once @trans is solved, just make a rot_collision identity matrix for no collision case
#    if b.jiggle_collision:
#        for c_item in b.jiggle_collider_list:
#            if bpy.context.scene.objects.find(c_item.name) >= 0:
#                collision_test = collide_bone(c_item, b, eulerRot)
#                if collision_test:
#                    rot_collision = collision_test
#            else: #collider has disappeared from jiggle list, recalculate
#                jiggle_list_refresh_ui(self,bpy.context)
#            
#        if rot_collision:
#            print(b["c_largest"])
#            b.jiggle_spring = Vector(b.jiggle_spring) + Vector((rot_collision.z,0,-rot_collision.x))
#            b.jiggle_velocity = Vector(b.jiggle_velocity) *0 #this needs to reflect off surface normal
#            #b.jiggle_velocity = 4*Vector((rot_collision.z,0,-rot_collision.x))
#            new_mat = b.matrix @ eulerRot.to_matrix().to_4x4() @ rot_collision.to_matrix().to_4x4()
#            #new_mat = b.matrix @ trans @ eulerRot.to_matrix().to_4x4()       
#        else:
#            new_mat = update_matrix @ b.matrix @ trans @ eulerRot.to_matrix().to_4x4()
#            
#        print(" ") #just to break up readout between frames
#    
#    else:
    
    #this is a scale multiplier on keyed bones, but need to account for jiggle pre state
    s = (1+(local_spring.translation.y*b.jiggle_stretch))
    s_mat = Matrix.Scale(s, 4, Vector((0,1,0)))  
    
    new_mat = b.matrix @ trans @ eulerRot.to_matrix().to_4x4() @ s_mat
    
    for c in b.constraints:
        if c.type == 'CHILD_OF' and not c.mute:
            new_mat = b.bone.matrix_local @ b.matrix_basis @ b.matrix.inverted() @ new_mat
            break

    b.matrix = new_mat
    
    #this becomes the new previous frame matrix: (this one needs parent updates in new_b_mat, where above uses pre-parent b.matrix)
    new_mat = new_b_mat @ trans @ eulerRot.to_matrix().to_4x4() @ s_mat
    b['jiggle_mat']=b.id_data.matrix_world @ new_mat
    
    return new_mat

#new tree based jiggle logic
def jiggle_tree_pre(jiggle_tree,ob=None):
    if bpy.context.scene.jiggle_enable:
        for item in jiggle_tree:
            if 'bones' in jiggle_tree[item]:
                #process objects
                if item in bpy.data.objects:
                    jiggle_tree_pre(jiggle_tree[item]['bones'], bpy.data.objects[item])
                    jiggle_tree_pre(jiggle_tree[item]['children'])
                else:
                    generate_jiggle_tree()
            else:
                #process bones
                if item in ob.pose.bones:
                    b = ob.pose.bones[item]
                    jiggle_bone_pre(b)
                    jiggle_tree_pre(jiggle_tree[item]['children'], ob)
                else:
                    generate_jiggle_tree()
                

#post assumes pre has ensured jiggle tree items exist?                  
def jiggle_tree_post2(jiggle_tree, ob=None, parent=None, new_parent_mat=None):
    if bpy.context.scene.jiggle_enable:
        
        if bpy.context.scene.jiggle_use_fps_scale:
            bpy.context.scene.jiggle_rate = bpy.context.scene.render.fps / bpy.context.scene.render.fps_base / bpy.context.scene.jiggle_base_fps
        else:
            bpy.context.scene.jiggle_rate = 1.0
            
        for item in jiggle_tree:
            if 'bones' in jiggle_tree[item]:
                jiggle_tree_post2(jiggle_tree[item]['bones'], bpy.data.objects[item])
                jiggle_tree_post2(jiggle_tree[item]['children'])
            else:
                b = ob.pose.bones[item]
                if parent: #b's matrix should be offset by parent offset if it has one
                    diff_mat = (b.matrix.inverted() @ parent.matrix).inverted() #b and parent are both pre-jiggle (no view_layer updates)
                    new_b_mat = new_parent_mat @ diff_mat
                else:
                    new_b_mat = b.matrix
                new_p_mat = jiggle_bone_post(b, new_b_mat) #jiggle_bone_post should be updated to do all calcs on new_b_mat
                jiggle_tree_post2(jiggle_tree[item]['children'], ob, b, new_p_mat)
                if ((bpy.context.scene.frame_current == bpy.context.scene.frame_start) and bpy.context.scene.jiggle_reset) or not b.jiggle_enable: #if not jiggle enabled, it not it the list so this seems pointless?
                    b['jiggle_mat']=b.id_data.matrix_world @ b.matrix
                    
def reset_jiggle_tree(jiggle_tree, ob=None):
    if bpy.context.scene.jiggle_enable:
        for item in jiggle_tree:
            if 'bones' in jiggle_tree[item]:
                #process objects
                if item in bpy.data.objects:
                    reset_jiggle_tree(jiggle_tree[item]['bones'], bpy.data.objects[item])
                    reset_jiggle_tree(jiggle_tree[item]['children'])
                else:
                    generate_jiggle_tree()
            else:
                #process bones
                if item in ob.pose.bones:
                    b = ob.pose.bones[item]
                    reset_bone(b)
                    reset_jiggle_tree(jiggle_tree[item]['children'], ob)
                else:
                    generate_jiggle_tree()
                    
@persistent
def jiggle_pre(self):
    global curframe
    try:
        jiggle_tree = bpy.context.scene['jiggle_tree'].to_dict()
    except:
        generate_jiggle_tree()
        jiggle_tree = bpy.context.scene['jiggle_tree'].to_dict()
    if (bpy.context.scene.frame_current != curframe):
        jiggle_tree_pre(jiggle_tree)

@persistent
def jiggle_post(self,depsgraph):
    global curframe
    if (depsgraph.view_layer.name == bpy.context.view_layer.name):
        curframe = bpy.context.scene.frame_current
        print("post %d %d" %(curframe, bpy.context.scene.frame_current))
        jiggle_tree = bpy.context.scene['jiggle_tree'].to_dict()
        jiggle_tree_post2(jiggle_tree)      
                
######## NEW STUFF ENDS #######################################################################
        
@persistent
def jiggle_render(self):
    global render
    render = True
    
@persistent
def render_post(self):
    global render
    render = False

def select_bones(bone_tree, ob):
    for bone in bone_tree:
        ob.pose.bones[bone].bone.select = True
        select_bones(bone_tree[bone]['children'], ob)

class reset_wiggle(bpy.types.Operator):
    """Reset wiggle physics"""
    bl_idname = "id.reset_wiggle"
    bl_label = "Reset Physics State"
    
    @classmethod
    def poll(cls,context):
        return True
    
    def execute(self,context):
        jiggle_tree = bpy.context.scene['jiggle_tree'].to_dict()
        reset_jiggle_tree(jiggle_tree)   
        return {'FINISHED'}

class select_wiggle_bones(bpy.types.Operator):
    """Select wiggle bones in this armature"""
    bl_idname = "id.select_wiggle"
    bl_label = "Select Wiggle Bones"
    
    @classmethod
    def poll(cls,context):
        return (context.object is not None and context.object.type == 'ARMATURE' and context.mode == 'POSE')
    
    def execute(self,context):
        bpy.ops.pose.select_all(action='DESELECT')
        ob = context.object
        jiggle_tree = bpy.context.scene['jiggle_tree'].to_dict()
        if ob.name in jiggle_tree:
            select_bones(jiggle_tree[ob.name]['bones'], ob)     
        return {'FINISHED'}
        
class bake_jiggle(bpy.types.Operator):
    """Bake wiggle dynamics on selected bones"""
    bl_idname = "id.bake_wiggle"
    bl_label = "Bake Wiggle"
    
    @classmethod
    def poll(cls, context):
        return True
    
    def execute(self,context):
        ob = context.object
        if context.scene.jiggle_bake_additive:
            if ob.animation_data:
                if ob.animation_data.action:
                    action = ob.animation_data.action
                    track = ob.animation_data.nla_tracks.new()
                    track.strips.new(action.name, action.frame_range[0], action)
                    ob.animation_data.action = None
            else:
                ob.animation_data.create()
                ob.animation_data.use_nla = True
            ob.animation_data.action_blend_type = 'ADD'
        else:
            if ob.animation_data:
                ob.animation_data.action_blend_type = 'REPLACE'
            
        if not context.scene.jiggle_reset:
            #prewarm loop
            for frame in range(context.scene.frame_start,context.scene.frame_end):
                context.scene.frame_set(frame)
                if frame == context.scene.frame_start:
                    bpy.ops.id.reset_wiggle()
                    
        #bake bones - start to end, active bones, don't clear constraints
        bpy.ops.nla.bake(frame_start = context.scene.frame_start, frame_end = context.scene.frame_end, visual_keying=True)
        
        #turn off dynamics according to bpy.context.scene.jiggle_disable_mask
        mask = context.scene.jiggle_disable_mask
        if mask == 'BONES':
            for b in bpy.context.selected_pose_bones:
                b.jiggle_enable = False
        elif mask == 'ARMATURE':
            context.object.data.jiggle_enable = False
        elif mask == 'SCENE':
            context.scene.jiggle_enable = False
        else:
            print("shouldn't get here")
        bpy.context.area.type = "PROPERTIES"
        return {'FINISHED'}
    
class JiggleBonePanel(bpy.types.Panel):
    bl_label = 'Wiggle Bone'
    bl_idname = 'OBJECT_PT_jiggle_panel'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'bone'
    
    @classmethod
    def poll(cls, context):
        return (context.object and context.object.type == 'ARMATURE' and context.object.data.bones.active)
    
    def draw_header(self,context):
        b = context.object.pose.bones[context.object.data.bones.active.name]
        self.layout.prop(b, 'jiggle_enable', text="")
        self.layout.enabled = context.object.data.jiggle_enable and context.scene.jiggle_enable
    
    def draw(self,context):
        layout = self.layout
        b = context.object.pose.bones[context.object.data.bones.active.name]
        #layout.prop(b, 'jiggle_enable')
        layout.use_property_split = True
        
        col = layout.column()
        col.enabled = b.jiggle_enable and context.object.data.jiggle_enable and context.scene.jiggle_enable

        col.prop(b, 'jiggle_active')
        col = col.column()
        if not context.object.data.jiggle_enable:
            col.label(text="ARMATURE DISABLED.")
            #col.label(text="See Armature Settings.")
        if not context.scene.jiggle_enable:
            col.label(text="SCENE DISABLED.")
        col.prop(b, 'jiggle_stiffness')
        col.prop(b,'jiggle_dampen')
        col.prop(b, 'jiggle_amplitude')
        col.prop(b, 'jiggle_translation')
        col.prop(b, 'jiggle_stretch')
        col.prop(b, 'jiggle_gravity')
        col = col.column()
        col.enabled = False
        col.prop(b, 'jiggle_collision',text="Collisions disabled during beta.")
        #col.enabled = b.jiggle_active
        #col = col.column()
        col.prop(b, 'jiggle_collision_margin')
        col.prop(b, 'jiggle_collision_friction')
        col.enabled = b.jiggle_collision
        layout.separator()
        
        col = layout.column()
        col.label(text="Global Wiggle Utilities:")
        col.operator("id.reset_wiggle")
        col.prop(context.scene, 'jiggle_reset')
        col.separator()
        col.operator("id.select_wiggle")
        col.operator("id.bake_wiggle")
        layout.prop(context.scene, 'jiggle_bake_additive')
        layout.prop(context.scene,"jiggle_disable_mask",text="Bake disables wiggle:")
        
class JiggleScenePanel(bpy.types.Panel):
    bl_label = 'Wiggle Scene'
    bl_idname = 'OBJECT_PT_jiggle_scene_panel'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'scene'
    
    def draw_header(self,context):
        self.layout.prop(context.scene, 'jiggle_enable', text="")
    
    def draw(self,context):
       layout = self.layout
       layout.use_property_split = True
       col = layout.column()
       col.prop(context.scene, 'jiggle_reset')
       col.prop(context.scene, 'jiggle_use_fps_scale')
       col = col.column()
       col.prop(context.scene, 'jiggle_base_fps')
       col.enabled = context.scene.jiggle_use_fps_scale
#        layout.prop(context.scene, 'jiggle_enable')

class JiggleArmaturePanel(bpy.types.Panel):
    bl_label = 'Wiggle Armature'
    bl_idname = 'OBJECT_PT_jiggle_armature_panel'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'data'
    
    @classmethod
    def poll(cls, context):
        return (context.object is not None and context.object.type == 'ARMATURE')
    
    def draw_header(self,context):
        self.layout.prop(context.object.data, 'jiggle_enable', text="")
        
    def draw(self,context):
        c = context.object
        #layout = self.layout()
        
class JiggleColliderPanel(bpy.types.Panel):
    bl_label = 'Wiggle Collider'
    bl_idname = 'OBJECT_PT_jiggle_collider_panel'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'object'
    
    @classmethod
    def poll(cls, context):
        return (context.object is not None and context.object.type == 'EMPTY')
    
    def draw_header(self,context):
        self.layout.prop(context.object, 'jiggle_collider_enable', text="")
    
    def draw(self,context):
        #layout = self.layout
        c = context.object
        #layout.prop(c, 'jiggle_collider_enable')
        
class jiggle_bone_item(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()
    
class jiggle_collider_item(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty()
    theta_last: bpy.props.FloatProperty()
    dir_last: bpy.props.FloatProperty()

def register():
    
    bpy.utils.register_class(jiggle_bone_item)
    bpy.utils.register_class(jiggle_collider_item)
    bpy.utils.register_class(JiggleBonePanel)
    bpy.utils.register_class(JiggleScenePanel)
    bpy.utils.register_class(JiggleArmaturePanel)
    bpy.utils.register_class(JiggleColliderPanel)
    bpy.utils.register_class(bake_jiggle)
    bpy.utils.register_class(reset_wiggle)
    bpy.utils.register_class(select_wiggle_bones)
    
    bpy.types.PoseBone.jiggle_spring = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    bpy.types.PoseBone.jiggle_velocity = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    
    bpy.types.PoseBone.jiggle_spring2 = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    bpy.types.PoseBone.jiggle_velocity2 = bpy.props.FloatVectorProperty(default=Vector((0,0,0)))
    
    bpy.types.Scene.jiggle_enable = bpy.props.BoolProperty(
        name = 'Enabled:',
        description = 'Global toggle for all jiggle bones',
        default = True,
        update = update_tree
    )
    bpy.types.Scene.jiggle_reset = bpy.props.BoolProperty(
        name = 'Reset on Loop',
        description = 'Jiggle physics reset when looping playback',
        default = True
    )
    bpy.types.Scene.jiggle_use_fps_scale = bpy.props.BoolProperty(
        name = 'Frame Rate Scaling',
        description = 'Physics rate scales to match frame rate',
        default = False
    )
    bpy.types.Scene.jiggle_base_fps = bpy.props.FloatProperty(
        name = 'Base Frame Rate',
        description = 'The physics frame rate to match',
        default = 24.0
    )
    bpy.types.Scene.jiggle_rate = bpy.props.FloatProperty(name='Rate',default=1.0)
    
    mask_enum = [
        ('SCENE','Scene','scene mask'),
        ('ARMATURE','Armature', 'armature mask'),
        ('BONES','Bones', 'bones mask')
    ]
    bpy.types.Scene.jiggle_disable_mask = bpy.props.EnumProperty(items = mask_enum, name="Disable", default='BONES', description='What to disable after baking')

    bpy.types.Scene.jiggle_bake_additive = bpy.props.BoolProperty(
        name = 'Additive Bake:',
        description = 'Push any current action to NLA and create additive jiggle on top',
        default = True
    )
    bpy.types.Armature.jiggle_enable = bpy.props.BoolProperty(
        name = 'Enabled:',
        description = 'Toggle Dynamic jiggle bones on this armature',
        default = True,
        update = update_tree
    )
    bpy.types.Scene.jiggle_list = bpy.props.CollectionProperty(type=jiggle_bone_item)
    bpy.types.Scene.jiggle_collider_list = bpy.props.CollectionProperty(type=jiggle_bone_item)
    bpy.types.Object.jiggle_list = bpy.props.CollectionProperty(type=jiggle_bone_item)
    bpy.types.PoseBone.jiggle_collider_list = bpy.props.CollectionProperty(type=jiggle_collider_item)
    bpy.types.Object.jiggle_collider_enable = bpy.props.BoolProperty(
        name = 'Enabled',
        description = 'Activate as jiggle bone collider',
        default = False,
        update = jiggle_list_refresh_ui
    )
    bpy.types.PoseBone.jiggle_enable = bpy.props.BoolProperty(
        name = 'Enabled',
        description = 'Enable jiggle on this bone',
        default = False,
        update = jiggle_list_refresh_ui
    )
    bpy.types.PoseBone.jiggle_active = bpy.props.BoolProperty(
        name='Active',
        description='Animate this toggle to temporarily disable jiggle',
        default=True,
        update=active_update
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
    bpy.types.PoseBone.jiggle_collision = bpy.props.BoolProperty(
        name = 'Collisions',
        description = 'Activate for collisions',
        default = False,
        update = collision_update
    )
    bpy.types.PoseBone.jiggle_collision_margin = bpy.props.FloatProperty(
        name = 'Collision Tip Margin:',
        description = 'Adds radius to bone-collider detection, helpful for bone chains',
        default = 0.4,
        update = margin_update
    )
    bpy.types.PoseBone.jiggle_collision_friction = bpy.props.FloatProperty(
        name = 'Collision Friction:',
        description = '0-1 range for frictionless to sticky collisions',
        default = 0.5,
        update = friction_update
    )
    
#    bpy.app.handlers.frame_change_pre.clear()
#    bpy.app.handlers.frame_change_post.clear()
#    bpy.app.handlers.render_pre.clear()
#    bpy.app.handlers.render_post.clear()
    
    bpy.app.handlers.frame_change_pre.append(jiggle_pre)
    bpy.app.handlers.frame_change_post.append(jiggle_post)
    bpy.app.handlers.render_pre.append(jiggle_render)
    bpy.app.handlers.render_post.append(render_post)

def unregister():
    bpy.utils.unregister_class(JiggleBonePanel)
    bpy.utils.unregister_class(JiggleScenePanel)
    bpy.utils.unregister_class(JiggleArmaturePanel)
    bpy.utils.unregister_class(JiggleColliderPanel)
    bpy.utils.unregister_class(jiggle_bone_item)
    bpy.utils.unregister_class(jiggle_collider_item)
    
    bpy.utils.unregister_class(bake_jiggle)
    bpy.utils.unregister_class(select_wiggle_bones)
    bpy.utils.unregister_class(reset_wiggle)
    
    bpy.app.handlers.frame_change_pre.remove(jiggle_pre)
    bpy.app.handlers.frame_change_post.remove(jiggle_post)
    bpy.app.handlers.render_pre.remove(jiggle_render)
    bpy.app.handlers.render_post.remove(render_post)

if __name__ == "__main__":
    register()

#1.4.4 CHANGELOG

#bugfix: translational jiggle not reset on startframe
#bugfix: bake operator didn't like channel index specified
#feature: active toggle useful for animating a preroll without jiggle
#feature: operator to select other enabled wiggle bones in armature 
#feature: frame rate scaling of jiggle physics
#feature: looping physics toggle

#1.5 CHANGELOG

#optimization: speedup by avoiding view_layer.updates() 

#bone stretching now properly matches jiggle dampening (it was always fully damped before)

#constraint logic:
#   matrix logic modified when presence of 'CHILD_OF' constraint detected.
#   note that constraint order makes a big difference in terms of whether things work. Eg:
#       -maintain volume must come after child of or it'll freak out
#       -track to must come before child of or results will be weird

#cleanup: cleared out some old code, much remains!

#b12:
#scale respects initial scale when jiggle enabled
#post bake disabling shouldn't mess up bone enabled states anymore
#additive is optional when baking wiggle

#TODO

#   -jiggle_tree needs to be a better stored variable
#   -can constraints still get passed jiggle motion? (tricky order of operation, maybe only childof?) [KINDA DONE]

#NEXT PRIORITIES

#   -investigate why renders don't work
#   -should only disable baked bones?
#   -y-stretch should jiggle [DONE]
#   -make sure y-stretch works with preserve volume constraint [DONE - with caveats]

#FUTURE COLLISION STUFF

#   -better collision margin (fake bone extension)
#   -collision bounce/velocity transfer
#   -look into division by zero (i think in friction code)
#   -look into frictions that seem to still pull bones incorrectly
#   -make collision work with stretchy bones and amplitude jiggle
#   -per bone collider collection option
#   -box collider for better torso/ponytail collision scenarios
#   -stretched sphere colliders? capsule colliders?

#lower priority:
#cleaner code for property updates (one function for multiple properties)
#look into property groupings again (we're already using for c_items, right?)
#any other cleanups