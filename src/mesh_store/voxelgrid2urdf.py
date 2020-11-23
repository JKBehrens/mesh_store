import os

import numpy as np
import trimesh
import rospkg

from odio_urdf import *


def save_image(mesh):
    # mesh = trimesh.primitives.Sphere()
    if isinstance(mesh, trimesh.Trimesh):
        scene = mesh.scene()
    elif isinstance(mesh, trimesh.Scene):
        scene = mesh

    data = scene.save_image()
    # if this doesn't work, try it with a visible window:
    # data = scene.save_image(visible=True)

    from PIL import Image
    rendered = Image.open(trimesh.util.wrap_as_stream(data))
    file = open('my_mesh.png', 'w+')
    file.write(data)
    file.close()



def link(mesh,robot_name, center_gravity,mass,I,material,geom_origin, has_inertia=True, has_visual=True, has_collision=True):
    """
        Most of the links are the same except for the passed in info.
        This function just allows grouping the important numbers better.
    """
    # N = str(N)
    assert isinstance(mesh, str)
    if robot_name is None:
        ret = Link(name = mesh.split('.')[0])
    else:
        ret = Link(name=robot_name)
    if has_inertia:
        ret(Inertial(
            Origin(list(center_gravity)),
            Mass(value=mass * 1000),
            Inertia(I)))
    if has_visual:
        ret(Visual(
            Origin(geom_origin),
            Geometry(Mesh(filename = os.path.join('package://mesh_store/meshes', mesh.replace(os.path.dirname(mesh),'').replace('/','')))),
            Material(material)))
    if has_collision:
        ret(Collision(
            Origin(geom_origin),
            Geometry(Mesh(filename = os.path.join('package://mesh_store/meshes', mesh.replace(os.path.dirname(mesh),'').replace('/','')))),
            Material(material)),
            Contact())

    return ret

def mesh2urdf(mesh_path, obj_name, scale=[0.1, 0.1, 0.1, 1.0], maxhulls=50, init_orig=[0.2, 0, 1.0, 0, 0, 0], robot_urdf=None):
    # attach to logger so trimesh messages will be printed to console
    # trimesh.util.attach_to_log()


    # obj_name = 'soda_can'
    if os.path.isfile(mesh_path):
        # mesh = trimesh.load(obj_name + '.stl')  # type: trimesh
        mesh = trimesh.load(mesh_path)  # type: trimesh
    elif 'package' in mesh_path:
        r = rospkg.RosPack()
        path = mesh_path.replace('package://','')
        pkg_name = path.split('/')[0]
        pack_path = r.get_path(pkg_name)
        abs_path = os.path.join(pack_path, path.replace(pkg_name,'')[1:])
        mesh = trimesh.load(abs_path)  # type: trimesh
        mesh_path = abs_path


    assert isinstance(mesh, trimesh.Trimesh)
    scale_mat = np.diag(scale)
    mesh = mesh.apply_transform(scale_mat)

    # save scaled mesh
    mesh_path_visual = os.path.join(os.path.dirname(mesh_path), obj_name + '_visual.stl')
    mesh.export(file_obj=mesh_path_visual, file_type='stl')

    # test if mesh has several components
    mesh_components = mesh.split(only_watertight=False)
    # for mesh_component in mesh_components:
    #     mesh_component.show()

    # make convex decomposition for simplified collision model
    bodies = []
    for mesh_component in mesh_components:
        assert isinstance(mesh_component, trimesh.Trimesh)
        print(mesh_component.bounds)
        print(mesh_component.scale)
        print(mesh_component.bounding_box.volume)
        # if mesh_component.bounding_box.volume < 1e-6 or mesh_component.scale < 0.0005 or len(mesh_component.vertices) < 10:
        if mesh_component.bounding_box.volume < 1e-5 or mesh_component.scale < 0.0005 or len(
                mesh_component.vertices) < 15:

            print('skipping mesh component because it is of negligible size')
            continue

        # mesh_component.show()
        decomp = mesh_component.convex_decomposition(maxhulls=maxhulls, pca=1, mode=1)
        if isinstance(decomp, list):
            bodies += decomp
        elif isinstance(decomp, trimesh.Trimesh):
            bodies.append(decomp)
            # decomp.show()
        print('.')

    # show the decomposition
    scene = trimesh.Scene()
    for body in bodies:
        scene.add_geometry(body)
    # scene.show()
    save_image(scene)

    if robot_urdf is None:
        myobj = Robot()
        obj_start_index = len(myobj)
    else:
        myobj = robot_urdf
        obj_start_index = len(myobj)
    # add visual link for RVIZ
    # mesh_path = obj_name + '.stl'
    link_vis = link(mesh_path_visual, obj_name + '_visual', None, None, None, "Grey", [0, 0, 0],
                has_collision=False,
                has_inertia=False,
                has_visual=True)
    myobj(link_vis)

    for idx, body in enumerate(bodies): # type: trimesh.Trimesh
        name = obj_name + '_' + '{:06d}'.format(idx)
        body_mesh_path = name + '.stl'
        # os.path.join(os.path.dirname(mesh_path), body_mesh_path)
        body.export(file_obj=os.path.join(os.path.dirname(mesh_path), body_mesh_path), file_type='stl')

        mass = body.mass
        inertia = body.moment_inertia
        # print(body.principal_inertia_components)

        ixx = inertia[0,0]
        iyy = inertia[1,1]
        izz = inertia[2,2]
        ixy = inertia[0,1]
        ixz = inertia[0,2]
        iyz = inertia[1,2]


        link4 = link(body_mesh_path, None, body.center_mass, mass, [ixx, ixy, ixz, iyy, iyz, izz], "Grey", [0, 0, 0])

        # print(link4)
        myobj(link4)

    # print(myobj)

    for idx, links in enumerate(zip(myobj[obj_start_index:-1], myobj[obj_start_index+1:])):
        l1, l2 = links
        print(l1.name)
        joint = Joint(obj_name + "_joint_{:04d}".format(idx), Parent(str(l1.name)), Child(str(l2.name)), type="fixed")
        myobj(joint)

    # joint = Joint("joint_world_" + obj_name, Parent('world'), Child(myobj[obj_start_index].name), type="floating")
    joint = Joint("joint_world_" + obj_name, Parent('base_link'), Child(myobj[obj_start_index].name), type="floating")
    joint(Origin(init_orig))
    myobj(joint)

    return myobj

    # print(myobj)
    # text_file = open(obj_name + ".urdf", "w")
    # n = text_file.write(myobj.__str__())
    # text_file.close()


if __name__ == '__main__':
    mesh2urdf('soda_can.stl', 'soda_can', scale=[0.08, 0.08, 0.08, 1.0], maxhulls=50)
    # mesh2urdf('kettle1.stl', 'kettle1', scale=[0.08, 0.08, 0.08, 1.0], maxhulls=20)
    mesh2urdf('wine_glass2.stl', 'wine_glass2', scale=[0.08, 0.08, 0.08, 1.0], maxhulls=50)


