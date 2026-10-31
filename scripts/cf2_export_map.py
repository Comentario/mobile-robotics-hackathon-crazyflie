"""Export measured occupancy as ROS map-server PGM/YAML, without flight commands."""
import argparse
import json
import math
from pathlib import Path


def export_map(snapshot, source_path):
    if snapshot.get('origin') is None or snapshot.get('start') is None:
        return []
    source_path = Path(source_path)
    res, width, height = snapshot['resolution'], snapshot['width'], snapshot['height']
    ox, oy = snapshot['origin']; sx, sy = snapshot['start']
    x0 = y0 = 0; x1, y1 = width, height
    area = snapshot.get('area')
    if area:
        # Keep only complete cells contained in the configured rectangle.
        x0 = max(0, math.ceil((sx-area[0]/2-ox)/res-1e-9))
        y0 = max(0, math.ceil((sy-area[1]/2-oy)/res-1e-9))
        x1 = min(width, math.floor((sx+area[0]/2-ox)/res+1e-9))
        y1 = min(height, math.floor((sy+area[1]/2-oy)/res+1e-9))
    if x1 <= x0 or y1 <= y0:
        return []
    layers = snapshot.get('layers') or [{'height_m': .3, 'data': snapshot['data']}]
    result, metadata = [], []
    for layer in layers:
        base = source_path.with_name(source_path.stem+f"_z{layer['height_m']:.2f}")
        pgm, yaml = base.with_name(base.name+'.pgm'), base.with_name(base.name+'.yaml')
        values = layer['data']
        # OccupancyGrid rows start at the bottom; image rows start at the top.
        pixels = bytes(205 if values[y*width+x] < 0 else
                       (0 if values[y*width+x] >= 50 else 254)
                       for y in range(y1-1,y0-1,-1) for x in range(x0,x1))
        with pgm.open('xb') as f:
            f.write(f'P5\n{x1-x0} {y1-y0}\n255\n'.encode()+pixels)
        origin = [ox+x0*res-sx, oy+y0*res-sy, 0.]
        with yaml.open('x') as f:
            f.write(f'image: {json.dumps(pgm.name)}\nmode: trinary\nresolution: {res}\n'
                    f'origin: {json.dumps(origin)}\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        metadata.append({'height_m': layer['height_m'], 'yaml': yaml.name,
            'image': pgm.name, 'origin_relative_to_start_m': origin,
            'width':x1-x0,'height':y1-y0})
        result.extend((pgm,yaml))
    manifest = source_path.with_name(source_path.stem+'_handoff.json')
    with manifest.open('x') as f:
        json.dump({'schema':'cf2.offline_map.v1','session_id':snapshot.get('session_id'),
            'source_snapshot':source_path.name,'resolution_m':res,
            'start_in_session_ekf_m':[sx,sy], 'coordinate_frame':'CF2 start-relative XY; session EKF axes',
            'start_xy_m':[0.,0.], 'layers':metadata,
            'unknown_policy':'Unknown cells are not traversable for CF1',
            'shared_frame_aligned':False,'map_validated_for_drone1':False,
            'requires':'Measured XY/yaw alignment to CF1 and obstacle/pose accuracy validation'},f,indent=2)
    result.append(manifest)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot',type=Path)
    args=parser.parse_args()
    for path in export_map(json.loads(args.snapshot.read_text()),args.snapshot): print(path)
