#!/usr/bin/env python

import os 
import argparse
import numpy as np 
import cv2 
import yaml
from lxml import etree as ET
from copy import copy



class WorldGenerator(object):
    kernel_size = 4 # for 2D map image preprocessing
    def __init__(self, yaml_path):
        self.yaml_path = yaml_path 
        self.pgm_path = os.path.join(os.path.dirname(yaml_path),
                                     os.path.splitext(os.path.basename(yaml_path))[0] + '.pgm')
        self.generated_world_path = os.path.join(os.path.dirname(os.path.dirname(yaml_path)),
                                                 'worlds',
                                                 'generated_' + os.path.splitext(os.path.basename(yaml_path))[0] + '.world')

        self.map = cv2.imread(self.pgm_path, -1).astype(np.float)

        self.map[self.map==0] = 255 # obstacle
        self.map[self.map==254] = 0 # empty 
        self.map[self.map==205] = 0 # unknown 

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size))
        self.map = cv2.morphologyEx(self.map, cv2.MORPH_CLOSE, kernel)

        with open(self.yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)

        self.origin = np.array([yaml_data['origin'][0],
                                yaml_data['origin'][1]])
        self.resolution = yaml_data['resolution'] 

    def write_basics(self):
        sdf = ET.Element("sdf",version="1.6")
        world = ET.SubElement(sdf, "world", name="default")
        # global light source
        include = ET.SubElement(world, 'include')
        uri = ET.SubElement(include, 'uri')
        uri.text = 'model://sun'
        # ground plane
        include = ET.SubElement(world, 'include')
        uri = ET.SubElement(include, 'uri')
        uri.text = 'model://ground_plane'
        # ode
        physics = ET.SubElement(world, 'physics', type='ode')
        real_time_update_rate = ET.SubElement(physics, 'real_time_update_rate')
        real_time_update_rate.text = '1000.0'
        max_step_size = ET.SubElement(physics, 'max_step_size')
        max_step_size.text = '0.001'
        real_time_factor = ET.SubElement(physics, 'real_time_factor')
        real_time_factor.text = '1'
        ode = ET.SubElement(physics, 'ode')
        solver = ET.SubElement(ode, 'solver')
        type = ET.SubElement(solver, 'type')
        type.text = 'quick'
        iters = ET.SubElement(solver, 'iters')
        iters.text = '150'
        precon_iters = ET.SubElement(solver, 'precon_iters')
        precon_iters.text = '0'
        sor = ET.SubElement(solver, 'sor')
        sor.text = '1.400000'
        use_dynamic_moi_rescaling = ET.SubElement(solver, 'use_dynamic_moi_rescaling')
        use_dynamic_moi_rescaling.text = '1'
        constraints = ET.SubElement(ode, 'constraints')
        cfm = ET.SubElement(constraints, 'cfm')
        cfm.text = '0.00001'
        erp = ET.SubElement(constraints, 'erp')
        erp.text = '0.2'
        contact_max_correcting_vel = ET.SubElement(constraints, 'contact_max_correcting_vel')
        contact_max_correcting_vel.text = '2000.000000'
        contact_surface_layer = ET.SubElement(constraints, 'contact_surface_layer')
        contact_surface_layer.text = '0.01000'

        # scene
        scene = ET.SubElement(world, 'scene')
        ambient = ET.SubElement(scene, 'ambient')
        ambient.text = '0.4 0.4 0.4 1'
        background = ET.SubElement(scene, 'background')
        background.text = '0.7 0.7 0.7 1'
        shadows = ET.SubElement(scene, 'shadows')
        shadows.text = 'true'

        # gui
        gui = ET.SubElement(world, 'gui', fullscreen='0')
        camera = ET.SubElement(gui, 'camera', name='user_camera')
        pose = ET.SubElement(camera, 'pose')
        pose.text = '0.0 0.0 17.0 -1.5708 1.5708 0'
        view_controller = ET.SubElement(camera, 'view_controller')
        view_controller.text ='orbit'

        return sdf, world

    def generate_v_wall(self, parent, x, y, length, id):
        collision = ET.SubElement(parent, 'collision', name='obstacle_{}'.format(id))
        pose = ET.SubElement(collision, 'pose')
        pose.text = '{} {} 0 0 0 1.5708'.format(str(x), str(y))
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        size = ET.SubElement(box, 'size')
        size.text = '{} 0.15 1.0'.format(str(length))

        visual = ET.SubElement(parent, 'visual', name='obstacle_{}'.format(id))
        pose = ET.SubElement(visual, 'pose')
        pose.text = '{} {} 0 0 0 1.5708'.format(str(x), str(y))
        geometry = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geometry, 'box')
        size = ET.SubElement(box, 'size')
        size.text = '{} 0.15 1.0'.format(str(length))
        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        uri = ET.SubElement(script, 'uri')
        uri.text = 'file://media/materials/scripts/gazebo.material'
        name = ET.SubElement(script, 'name')
        name.text = 'Gazebo/Grey'
        ambient = ET.SubElement(material, 'ambient')
        ambient.text = '1 1 1 1'

    def generate_h_wall(self, parent, x, y, length, id):
        collision = ET.SubElement(parent, 'collision', name='obstacle_{}'.format(id))
        pose = ET.SubElement(collision, 'pose')
        pose.text = '{} {} 0 0 0 0'.format(str(x), str(y))
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        size = ET.SubElement(box, 'size')
        size.text = '{} 0.15 1.0'.format(str(length))

        visual = ET.SubElement(parent, 'visual', name='obstacle_{}'.format(id))
        pose = ET.SubElement(visual, 'pose')
        pose.text = '{} {} 0 0 0 0'.format(str(x), str(y))
        geometry = ET.SubElement(visual, 'geometry')
        box = ET.SubElement(geometry, 'box')
        size = ET.SubElement(box, 'size')
        size.text = '{} 0.15 1.0'.format(str(length))
        # cylinder = ET.SubElement(geometry, 'cylinder')
        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        uri = ET.SubElement(script, 'uri')
        uri.text = 'file://media/materials/scripts/gazebo.material'
        name = ET.SubElement(script, 'name')
        name.text = 'Gazebo/Grey'
        ambient = ET.SubElement(material, 'ambient')
        ambient.text = '1 1 1 1'
  
    def is_v_wall_type(self, r, c, gridmap):
        """
            Return True if vertical wall is proper to ...

            This function helps to reduce the number of walls to generate
        """

        vl = 0 # expected vertical length
        rr = r
        cc = c
        while self.map[rr][cc] == 255:
            rr += 1
            vl += 1
            
        hl = 0 # expected horizontal length
        rr = r
        cc = c
        while self.map[rr][cc] == 255:
            cc += 1
            hl += 1
        return vl > hl

    def generate(self):
        sdf, world = self.write_basics()

        model = ET.SubElement(world, 'model', name='obstacle')
        static = ET.SubElement(model, 'static')
        static.text = '1'
        pose = ET.SubElement(model, 'pose', frame='')
        pose.text = '0 0 0 0 0 0'
        link = ET.SubElement(model, 'link', name='obstacle')
        
        parent = link

        flag = copy(self.map) # 255 -> need to build wall, 0 -> already ...

        wall_count = 0
        while True:
            idxs = np.argwhere(flag == 255)
            if len(idxs) == 0:
                break

            wall_count += 1

            start_r, start_c = idxs[0]
                
            h_wall = False
            v_wall = False
            end_r = start_r
            end_c = start_c
            # if self.map[end_r + 1][end_c] == 255 :
            if self.is_v_wall_type(start_r, start_c, self.map):
                v_wall = True
                while self.map[end_r][end_c] == 255:
                    flag[end_r][end_c] = 0
                    end_r += 1
                center_x = self.origin[0] + int((start_c + end_c) / 2) * self.resolution
                center_y = self.origin[1] + int((start_r + end_r) / 2) * self.resolution
                l = (end_r - start_r) * self.resolution
                self.generate_v_wall(link, center_x, center_y, l, wall_count)
            elif self.map[end_r][end_c + 1] == 255:
                h_wall = True
                while self.map[end_r][end_c] == 255:
                    flag[end_r][end_c] = 0
                    end_c += 1
                center_x = self.origin[0] + int((start_c + end_c) / 2) * self.resolution
                center_y = self.origin[1] + int((start_r + end_r) / 2) * self.resolution
                l = (end_c - start_c) * self.resolution
                self.generate_h_wall(link, center_x, center_y, l, wall_count)
            else:
                flag[end_r][end_c] = 0
                continue
            print('[{} idxs remained], wall_count={}, center_x={}, center_y={}, l={}'.format(len(idxs), wall_count, center_x, center_y, l))

        tree = ET.ElementTree(sdf)
        tree.write(self.generated_world_path, pretty_print=True)

parser = argparse.ArgumentParser(description="Run commands")
parser.add_argument('--yaml_path', type=str, default="./maps/2020-02-13-14-36-38.yaml", 
                    help="yaml path")
args = parser.parse_args()

world_generator = WorldGenerator(args.yaml_path)
world_generator.generate()

