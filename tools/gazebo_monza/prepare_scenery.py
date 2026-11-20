"""Prepare an offline circuit scenery snapshot from OSM 0.6 map XML and a WGS84 GP route.

Developer preprocessing only (Shapely >= 2.1). The Gazebo container reads the saved GeoJSON.
"""
import argparse
from collections import defaultdict
from datetime import date
from hashlib import sha256
import json
from math import cos, dist, floor, radians
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from shapely import constrained_delaunay_triangles
from shapely.affinity import affine_transform
from shapely.geometry import LineString, MultiLineString, Point, Polygon, mapping, shape
from shapely.ops import linemerge, split, unary_union


TAG_KEYS = ("name", "name:en", "building", "building:levels", "height", "covered", "roof:shape",
            "landuse", "natural", "water", "waterway", "leisure", "sport", "surface", "genus", "species",
            "man_made", "tower:type", "building:use", "highway", "service", "raceway", "width")
LICENSE = "https://opendatacommons.org/licenses/odbl/1-0/"
SOURCE = "https://api.openstreetmap.org/api/0.6/map?bbox="


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def kind_for(element, values, desert):
    building = values.get("building")
    name = values.get("name", "")
    if element.tag == "node":
        if (values.get("man_made") == "tower" and values.get("tower:type") == "observation"
                and height_m(values) is not None):
            return "observation_tower"
        return "tree" if values.get("natural") == "tree" else None
    if building == "grandstand" or values.get("leisure") == "grandstand":
        return "grandstand"
    if building and re.search(r"(?:^|\s)(?:grandstand|tribun[ae]|tribüne|trybuna)\b", name, re.I):
        return "grandstand"
    if building and building not in ("no", "ruins", "construction"):
        if re.search(r"\b(?:pit(?:s)?|paddocks?|boxengasse|boxengebäude|pitgebouw|boxes)\b", name, re.I):
            return "pit_building"
        return "building"
    if values.get("landuse") == "forest" or values.get("natural") == "wood":
        return "woodland"
    if values.get("natural") == "water" or values.get("waterway") == "riverbank":
        return "water"
    if values.get("natural") == "sand":
        return "sand"
    if desert and values.get("landuse") in ("grass", "meadow", "village_green"):
        return "grass"
    return None


def parts(geometry):
    if geometry.geom_type == "Polygon":
        if not geometry.is_empty:
            yield geometry
    elif geometry.geom_type in ("MultiPolygon", "GeometryCollection"):
        for member in geometry.geoms:
            yield from parts(member)


def no_holes(polygon):
    """Split surface rings across a courtyard/island for exterior-only mesh indices."""
    if not polygon.interiors:
        yield polygon
        return
    y = Polygon(polygon.interiors[0]).representative_point().y
    west, _, east, _ = polygon.bounds
    cutter = LineString([(west - 1, y), (east + 1, y)])
    for part in parts(split(polygon, cutter)):
        if len(part.interiors) >= len(polygon.interiors):
            raise ValueError("Could not split inner ring")
        yield from no_holes(part)


class OsmMap:
    def __init__(self, file):
        self.root = ET.parse(file).getroot()
        self.nodes = {n.get("id"): (float(n.get("lon")), float(n.get("lat"))) for n in self.root.findall("node")}
        self.ways = {w.get("id"): w for w in self.root.findall("way")}

    def coords(self, way):
        return [self.nodes[nd.get("ref")] for nd in way.findall("nd")]

    def polygon(self, element):
        if element.tag == "way":
            points = self.coords(element)
            return Polygon(points) if len(points) >= 4 and points[0] == points[-1] else None

        groups = defaultdict(list)
        for member in element.findall("member"):
            if member.get("type") != "way" or member.get("role") not in ("", "outer", "inner"):
                continue
            way = self.ways.get(member.get("ref"))
            if way is None:  # A bbox response can include an incomplete relation.
                return None
            groups[member.get("role") or "outer"].append(self.coords(way))

        def assembled(pieces):
            if not pieces:
                return Polygon()
            merged = linemerge(MultiLineString(pieces))
            rings = merged.geoms if merged.geom_type == "MultiLineString" else [merged]
            if any(not ring.is_ring or len(ring.coords) < 4 for ring in rings):
                return None
            areas = [Polygon(ring) for ring in rings]
            return unary_union(areas) if all(a.is_valid for a in areas) else None

        exterior, holes = assembled(groups["outer"]), assembled(groups["inner"])
        return exterior.difference(holes) if exterior is not None and holes is not None else None


class LocalFrame:
    """Local metric plane around the course. Output coordinates retain WGS84."""
    def __init__(self, lon, lat):
        xscale = 111_320 * cos(radians(lat))
        yscale = 111_132
        self.forward = (xscale, 0, 0, yscale, -xscale * lon, -yscale * lat)
        self.inverse = (1 / xscale, 0, 0, 1 / yscale, lon, lat)

    def local(self, shape):
        return affine_transform(shape, self.forward)

    def lonlat(self, shape):
        return affine_transform(shape, self.inverse)


def height_m(values):
    match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*(?:m)?", values.get("height", ""), re.I)
    if match and 1 <= float(match[1]) <= 350:
        return float(match[1])
    return None


def pit_lanes(source, relation_id, frame, origin):
    """Use the GP relation's pit members or explicitly identified ways at the starting grid."""
    relation = source.root.find(f"./relation[@id='{relation_id}']")
    pit_roles = {"pit_lane", "pitlane", "pit lane"}
    members = [] if relation is None else [
        member for member in relation.findall("member")
        if member.get("type") == "way" and member.get("role") in pit_roles
    ]
    if members:
        ways = [source.ways[m.get("ref")] for m in members if m.get("ref") in source.ways]
        evidence = {"pit_relation_id": int(relation_id), "pit_relation_version": int(relation.get("version"))}
    else:
        # A venue may have kart, club or disused pit lanes in the same bbox.
        # Follow the named pit nearest the starting grid and adjoining end-to-end segments.
        names = {"pit lane", "pitstraat", "boxengasse", "boxenstraße"}
        candidates = []
        for way in source.ways.values():
            t = tags(way)
            if t.get("highway") not in {"service", "raceway"}:
                continue
            if (t.get("raceway") not in pit_roles and t.get("service") not in pit_roles
                    and not names.intersection(t.get(k, "").casefold() for k in ("name", "name:en"))):
                continue
            coordinates = source.coords(way)
            distance = frame.local(LineString(coordinates)).distance(origin)
            candidates.append((way, distance, {coordinates[0], coordinates[-1]}))
        if not candidates:
            return []
        first, distance, ends = min(candidates, key=lambda row: row[1])
        if distance > 75:
            return []
        ways = [first]
        pending = [row for row in candidates if row[0] is not first]
        while connected := [row for row in pending if ends.intersection(row[2])]:
            for row in connected:
                ways.append(row[0])
                ends.update(row[2])
                pending.remove(row)
        evidence = {"pit_evidence": "mapped pit lane by the starting grid"}

    selected = []
    for way in ways:
        t = tags(way)
        if t.get("tunnel", "no") != "no" or t.get("bridge", "no") != "no" or t.get("layer", "").startswith("-"):
            continue
        # Merge entrances and exits visually on the main track. Neither width nor side markings are surveyed.
        geometry = frame.local(LineString(source.coords(way)))
        selected.append((way, t, geometry, evidence))
    return selected


def area_triangles(polygon):
    vertices = {point: i for i, point in enumerate(list(polygon.exterior.coords)[:-1])}
    cells = constrained_delaunay_triangles(polygon).geoms
    if abs(sum(cell.area for cell in cells) - polygon.area) > max(1e-16, polygon.area * 1e-7):
        raise ValueError("Incomplete polygon triangulation")
    return [[vertices[point] for point in list(cell.exterior.coords)[:-1]] for cell in cells]


def stand_rectangles(polygon, course, pavement, depth=0):
    """Fit steps to straight portions of curved stands. Split at most twice."""
    rectangle = polygon.minimum_rotated_rectangle
    corners = list(rectangle.exterior.coords)[:-1]
    if len(corners) != 4:
        return
    lengths = [dist(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    i = lengths.index(max(lengths))
    if min(lengths) < 2.2 or max(lengths) < 5:
        return
    # Curved stands often enclose a large empty area in a single fitted rectangle.
    if (polygon.area / rectangle.area < 0.83 or rectangle.intersects(pavement)) and depth < 2:
        m = LineString((corners[i], corners[(i + 1) % 4])).interpolate(0.5, normalized=True)
        n = LineString((corners[(i + 3) % 4], corners[(i + 2) % 4])).interpolate(0.5, normalized=True)
        dx, dy = n.x - m.x, n.y - m.y
        cutter = LineString([(m.x - dx, m.y - dy), (n.x + dx, n.y + dy)])
        for piece in parts(split(polygon, cutter)):
            yield from stand_rectangles(piece, course, pavement, depth + 1)
        return
    if rectangle.intersects(pavement):
        return
    faces = (i, (i + 2) % 4)
    front = min(faces, key=lambda j: course.distance(
        LineString((corners[j], corners[(j + 1) % 4])).interpolate(0.5, normalized=True)))
    yield polygon, corners[front:] + corners[:front]


def rounded(value):
    if isinstance(value, dict):
        return {k: rounded(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [rounded(v) for v in value]
    return round(value, 8) if isinstance(value, float) else value


def prepare(course_id, route, width, osm_file, retrieval, output, gp_relation=None):
    source = OsmMap(osm_file)
    frame = LocalFrame(*route[0][:2])
    course = frame.local(LineString([p[:2] for p in route]))
    road = course.buffer(width / 2 + 0.8, quad_segs=4)
    surroundings = course.buffer(190, quad_segs=5)
    desert = course_id in {"sakhir", "yasmarina"}
    elements = list(source.root.findall("relation")) + list(source.ways.values())
    candidates = []
    covered_ways = set()
    for e in elements:
        t = tags(e)
        k = kind_for(e, t, desert)
        if not k or (e.tag == "way" and (k, e.get("id")) in covered_ways):
            continue
        geo = source.polygon(e)
        if geo is None or geo.is_empty or not geo.is_valid:
            continue
        if e.tag == "relation":
            covered_ways.update((k, m.get("ref")) for m in e.findall("member") if m.get("role") != "inner")
        for part in parts(frame.local(geo)):
            gap = part.distance(course)
            if gap <= 190:
                candidates.append((k, e, t, part, gap))

    features = []
    counts = defaultdict(int)

    def add(kind, element, values, geometry, **attrs):
        osm_id = int(element.get("id"))
        key = element.tag, osm_id
        counts[key] += 1
        properties = {"kind": kind, "osm_type": element.tag, "osm_id": osm_id,
                      "version": int(element.get("version")), "timestamp": element.get("timestamp"),
                      "tags": {k: values[k] for k in TAG_KEYS if k in values}, **attrs}
        if (height := height_m(values)) is not None:
            properties["height_m"] = height
        features.append({
            "type": "Feature",
            "id": f"{element.tag}/{osm_id}/{counts[key]}",
            "properties": properties,
            "geometry": rounded(mapping(frame.lonlat(geometry))),
        })

    # Melbourne uses the GP driving line through public junctions but retains the mapped pit relation.
    pit_relation = 280443 if course_id == "melbourne" else gp_relation
    lanes = pit_lanes(source, pit_relation, frame, Point(course.coords[0]))
    solids = []
    for way, values, lane, evidence in lanes:
        add("pit_lane", way, values, lane, **evidence)
        solids.append(lane.buffer(3.25))
    pavement = unary_union([road, *solids])
    # Prefer actual trackside facades. A distant urban neighborhood can contain thousands of houses.
    buildings = sorted((item for item in candidates if item[0] == "building" and item[4] < 115
                        and item[3].area >= 22 and not item[3].intersects(road)), key=lambda item: item[4])
    selected_buildings = {(e.tag, e.get("id")) for _, e, _, _, _ in buildings[:150]}
    for k, e, t, area, _ in candidates:
        if k in ("grandstand", "pit_building", "building"):
            if area.area < 15 or (k == "building" and (e.tag, e.get("id")) not in selected_buildings) or area.intersects(road):
                continue
            solids.append(area)
            if k == "grandstand":
                for part, corners in stand_rectangles(area, course, pavement):
                    fitted = [frame.lonlat(Point(p)).coords[0] for p in corners]
                    add(k, e, t, part, model_corners=rounded(fitted), covered=t.get("covered") == "yes")
                continue
            projected = area.simplify(0.25)
        elif k in ("water", "sand", "grass"):
            projected = area.intersection(surroundings).difference(road).simplify(0.55)
            solids.append(projected)
        else:
            continue
        for piece in parts(projected):
            if piece.area < 8:
                continue
            for fragment in no_holes(piece):
                geo = shape(rounded(mapping(frame.lonlat(fragment))))
                if not geo.is_valid or geo.area < 1e-12:
                    continue
                add(k, e, t, fragment, triangles=area_triangles(geo))

    # Exclude track, buildings, water and runoff before planting generated trees.
    tracks = [frame.local(LineString(source.coords(w))).buffer(5)
              for w in source.ways.values() if tags(w).get("highway") == "raceway"]
    exclusion = unary_union([road.buffer(2), *tracks, *solids])
    plantable = course.buffer(135, quad_segs=4).difference(exclusion)
    forests = []
    for k, e, t, area, _ in candidates:
        if k != "woodland":
            continue
        wood = area.intersection(plantable)
        for part in parts(wood):
            clean = part.simplify(1.0).difference(exclusion)
            for ring in parts(clean):
                if ring.area > 85:
                    add(k, e, t, ring, tree_spacing_rc_m=3.0)
                    forests.append(ring)
        plantable = plantable.difference(wood)

    # Retain mapped street / park trees to give exposed circuits authentic tree rows.
    occupied = defaultdict(list)
    no_tree = unary_union([exclusion.buffer(7.5), *forests])
    for e in source.root.findall("node"):
        t = tags(e)
        kind = kind_for(e, t, desert)
        if kind not in ("tree", "observation_tower"):
            continue
        point = frame.local(Point(source.nodes[e.get("id")]))
        if kind == "observation_tower":
            if course.distance(point) <= 190 and not road.covers(point):
                add(kind, e, t, point)
            continue
        if course.distance(point) > 120 or no_tree.covers(point):
            continue
        x, y = point.coords[0]
        cx, cy = floor(x / 12), floor(y / 12)
        if any(dist((x, y), p) < 12 for ix in range(cx - 1, cx + 2) for iy in range(cy - 1, cy + 2)
               for p in occupied[(ix, iy)]):
            continue
        occupied[(cx, cy)].append((x, y))
        add("tree", e, t, point)
    bounds = source.root.find("bounds")
    bbox = [bounds.get(k) for k in ("minlon", "minlat", "maxlon", "maxlat")]
    metadata = {
        "type": "FeatureCollection",
        "name": f"{course_id} trackside scenery",
        "circuit_id": course_id,
        "attribution": "© OpenStreetMap contributors",
        "license": LICENSE,
        "source": SOURCE + ",".join(bbox),
        "osm_bbox_wgs84": [float(x) for x in bbox],
        "retrieved": retrieval,
        "source_xml_sha256": sha256(osm_file.read_bytes()).hexdigest(),
    }
    # One feature per line keeps the checked-in geographic snapshot reviewable.
    header = json.dumps(metadata, ensure_ascii=False, indent=2)[:-2]
    rows = [json.dumps(f, ensure_ascii=False, separators=(",", ":")) for f in features]
    output.parent.mkdir(exist_ok=True, parents=True)
    output.write_text(header + ',\n  "features": [\n    ' + ",\n    ".join(rows) + "\n  ]\n}\n")
    return features


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("track", type=Path, help="Circuit track.geojson containing the closed GP route")
    parser.add_argument("osm", type=Path, help="Geographical map XML from the OSM bbox API")
    parser.add_argument("--retrieved", type=date.fromisoformat, required=True)
    args = parser.parse_args()
    data = json.loads(args.track.read_text())
    line = next(f for f in data["features"] if f["properties"]["kind"] == "course")
    prepare(data["circuit_id"], line["geometry"]["coordinates"], line["properties"]["width_m"],
            args.osm, str(args.retrieved), args.track.with_name("scenery.geojson"),
            line["properties"].get("osm_id"))
