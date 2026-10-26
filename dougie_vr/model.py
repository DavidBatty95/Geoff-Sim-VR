"""Build a fixed-base arm rig from the original GeoffMain G1 MJCF."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'assets/g1/scene.xml'
ARM_PARTS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
             'wrist_roll', 'wrist_pitch', 'wrist_yaw')

def build():
    tree = ET.parse(ROOT / 'assets/g1/original.xml')
    root = tree.getroot()
    for tag in ('actuator', 'sensor', 'keyframe', 'visual'):
        for child in root.findall(tag):
            root.remove(child)
    for extra_world in root.findall('worldbody')[1:]:
        root.remove(extra_world)
    arms = {f'{side}_{part}_joint' for side in ('left', 'right') for part in ARM_PARTS}
    for body in root.findall('.//body'):
        for joint in body.findall('joint'):
            if joint.get('name') not in arms:
                body.remove(joint)
            else:
                joint.set('damping', '1.5')
                joint.set('armature', '0.02')
        # Arm-only commissioning rig: collisions are visual, not a grasp benchmark.
        for geom in body.findall('geom'):
            if geom.get('type') == 'mesh' and geom.get('group') != '1' and any(
                    other.get('group') == '1' and other.get('mesh') == geom.get('mesh')
                    for other in body.findall('geom')):
                body.remove(geom)
                continue
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast', gravity='0 0 -9.81')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    ET.SubElement(visual, 'headlight', ambient='0.5 0.5 0.5', diffuse='0.7 0.7 0.7')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='0 -2 3', dir='0 0 -1', diffuse='0.8 0.8 0.8')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='3 3 0.1', rgba='0.12 0.17 0.24 1')
    ET.SubElement(world, 'geom', name='table', type='box', pos='0.65 0 0.62', size='0.3 0.65 0.025', rgba='0.3 0.36 0.45 1')
    for y, color in ((0.25, '0 0.8 1 1'), (-0.25, '1 0.45 0.1 1')):
        ET.SubElement(world, 'geom', type='box', pos=f'0.6 {y} 0.70', size='0.04 0.04 0.055', rgba=color)
    for side, y in (('left', 0.032), ('right', -0.032)):
        # Camera local -Z points forward/down; +Y points up. Genuine stereo pair.
        ET.SubElement(world, 'camera', name=f'eye_{side}', pos=f'0.08 {y} 1.32',
                      xyaxes='0 -1 0 0.7660444 0 0.6427876', fovy='75')
        wrist = root.find(f'.//body[@name="{side}_wrist_yaw_link"]')
        ET.SubElement(wrist, 'site', name=f'{side}_tool', pos='0.04 0 0', size='0.008', rgba='0 0.8 1 1')
    actuator = ET.SubElement(root, 'actuator')
    for joint in root.findall('.//joint'):
        ET.SubElement(actuator, 'position', name=joint.get('name'), joint=joint.get('name'),
                      kp='80', kv='8', ctrlrange=joint.get('range'), forcerange='-25 25')
    ET.indent(tree)
    tree.write(MODEL, encoding='unicode')
    return MODEL

if __name__ == '__main__':
    print(build())
