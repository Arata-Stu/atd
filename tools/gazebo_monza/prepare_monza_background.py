"""Prepare Monza's outer tree belts and building footprints from an OSM map XML (Shapely 2.1+)."""
import argparse
from datetime import date
from hashlib import sha256
import json
from pathlib import Path

from shapely.affinity import affine_transform
from shapely.geometry import LineString, Polygon, box, mapping, shape
from shapely.ops import unary_union

from geography import ORIGIN_LAT, ORIGIN_LON, load_track, project
from prepare_scenery import LICENSE, SOURCE, TAG_KEYS, OsmMap, area_triangles, height_m, kind_for
from prepare_scenery import no_holes, parts, rounded, tags
from scenery import SCENERY_FILE, SCENERY_LAP_FILE


def prepare(osm, retrieved, output):
    source = OsmMap(osm)
    bounds = source.root.find('bounds')
    bbox = [float(bounds.get(key)) for key in ('minlon', 'minlat', 'maxlon', 'maxlat')]
    east = project(ORIGIN_LON + 1, ORIGIN_LAT)[0]
    north = project(ORIGIN_LON, ORIGIN_LAT + 1)[1]

    def local(geometry):
        return affine_transform(geometry, (east, 0, 0, north, -east * ORIGIN_LON, -north * ORIGIN_LAT))

    def lonlat(geometry):
        return affine_transform(geometry, (1/east, 0, 0, 1/north, ORIGIN_LON, ORIGIN_LAT))

    edges, _ = load_track()
    road = Polygon(edges['left']).difference(Polygon(edges['right']))
    prior = [f for path in (SCENERY_FILE, SCENERY_LAP_FILE) for f in json.loads(path.read_bytes())['features']]
    existing = {f['id'] for f in prior}
    obstacles = [local(shape(f['geometry'])) for f in prior if f['geometry']['type'].endswith('Polygon')]
    forests, features, members = [], [], set()

    def add(element, values, kind, geometry, **extra):
        identifier = f"{element.tag}/{element.get('id')}"
        props = {'kind': kind, 'osm_type': element.tag, 'osm_id': int(element.get('id')),
                 'version': int(element.get('version')), 'timestamp': element.get('timestamp'),
                 'tags': {key: values[key] for key in TAG_KEYS if key in values}, **extra}
        features.append({'type': 'Feature', 'id': f'{identifier}/background/{len(features)+1}',
                         'properties': props, 'geometry': rounded(mapping(geometry))})

    limit = local(box(*bbox))
    for element in [*source.root.findall('relation'), *source.ways.values()]:
        values = tags(element)
        way_id = element.get('id')
        if element.tag == 'way' and values.get('highway'):
            # Keep forest proxies away from mapped paths and other raceways.
            width = .45 if values['highway'] == 'raceway' else .22
            obstacles.append(local(LineString(source.coords(element))).buffer(width, quad_segs=2))
        kind = kind_for(element, values, desert=False)
        if kind not in ('woodland', 'building', 'pit_building', 'grandstand', 'water', 'sand'):
            continue
        if element.tag == 'way' and (kind, way_id) in members:
            continue
        geo = source.polygon(element)
        if geo is None or not geo.is_valid or geo.is_empty:
            continue
        if element.tag == 'relation':
            members.update((kind, m.get('ref')) for m in element.findall('member') if m.get('role') != 'inner')
        for area in parts(local(geo)):
            if kind == 'woodland':
                forests.append((element, values, area))
                continue
            obstacles.append(area)
            if kind != 'building' or f'{element.tag}/{way_id}' in existing or not limit.covers(area):
                continue
            gap = road.distance(area)
            if not 10 <= gap <= 100 or area.area < (1 if gap < 30 else 3):
                continue
            attrs = {}
            if (height := height_m(values)) is not None:
                attrs['height_m'] = height
            elif values.get('building:levels', '').isdigit() and int(values['building:levels']) > 0:
                attrs.update(height_m=3 * int(values['building:levels']), height_source='3 m per mapped level')
            for fragment in no_holes(area):
                polygon = shape(rounded(mapping(lonlat(fragment))))
                add(element, values, 'building', polygon, triangles=area_triangles(polygon), **attrs)

    # Preserve clearance after rounding the GeoJSON coordinates to eight decimal places.
    excluded = unary_union(obstacles).buffer(.001, quad_segs=2)
    for inner, outer, spacing in ((10, 35, 4.5), (35, 100, 7.0)):
        zone = limit.intersection(road.buffer(outer)).difference(road.buffer(inner)).difference(excluded)
        for element, values, area in forests:
            addition = zone.intersection(area)
            for fragment in parts(addition):
                if fragment.area >= spacing ** 2:
                    add(element, values, 'woodland', lonlat(fragment), tree_spacing_rc_m=spacing)
            zone = zone.difference(area)

    metadata = {'type': 'FeatureCollection', 'name': 'Monza GP circuit background', 'retrieved': str(retrieved),
                'source': SOURCE + ','.join(map(str, bbox)), 'source_xml_sha256': sha256(osm.read_bytes()).hexdigest(),
                'attribution': '© OpenStreetMap contributors', 'license': LICENSE, 'osm_bbox_wgs84': bbox}
    header = json.dumps(metadata, ensure_ascii=False, indent=2)[:-2]
    rows = [json.dumps(f, ensure_ascii=False, separators=(',', ':')) for f in features]
    output.write_text(header + ',\n  "features": [\n    ' + ',\n    '.join(rows) + '\n  ]\n}\n')


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('osm', type=Path)
    cli.add_argument('--retrieved', type=date.fromisoformat, required=True)
    cli.add_argument('--output', type=Path, default=SCENERY_FILE.with_name('monza_background.geojson'))
    args = cli.parse_args()
    prepare(args.osm, args.retrieved, args.output)
