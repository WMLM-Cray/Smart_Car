"""把 track_generator 出的赛道 world 补成 racecar_sim 能直接加载的 world。

差的只有一样:`libgazebo_ros_state.so`。它是 WorldPlugin,只能写在 world 文件里,
不能用 gzserver -s 加载(队友在 racecar.world 的注释里记了这个坑)。少了它
/gazebo/get_entity_state 没有,他们那套"世界系真实位姿对账里程计"的验收就跑不了。

用法:python 3Dsim/make_sim_world.py runs/track_s3/track_s3.world runs/track_s3/track_s3_sim.world
"""
import sys
import xml.etree.ElementTree as ET

PLUGIN = 'libgazebo_ros_state.so'


def main(src, dst):
    tree = ET.parse(src)
    world = tree.getroot().find('world')
    if world is None:
        raise SystemExit(f'{src} 里没有 <world>')
    if any(p.get('filename') == PLUGIN for p in world.findall('plugin')):
        raise SystemExit(f'{src} 已经有 {PLUGIN} 了')
    ET.SubElement(world, 'plugin', name='gazebo_ros_state', filename=PLUGIN)
    ET.indent(tree)
    tree.write(dst, encoding='utf-8', xml_declaration=True)
    print(f'{src} -> {dst}  (world={world.get("name")},'
          f' {len(world.findall("model"))} 个模型,已补 {PLUGIN})')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
