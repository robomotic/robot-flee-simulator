import json
import requests
from shapely.geometry import LineString, Polygon, mapping

def fetch_osm(bbox):
    s, w, n, e = bbox

    query = f'[out:json];(way["highway"]({s},{w},{n},{e});way["building"]({s},{w},{n},{e}););out geom;'

    url = "https://overpass-api.de/api/interpreter"
    headers = {"User-Agent": "curl/7.68.0"}
    response = requests.post(url, data={"data": query}, headers=headers)
    response.raise_for_status()
    return response.json()

def osm_to_geojson(data):
    features = []

    for el in data["elements"]:
        if "geometry" not in el:
            continue

        coords = [(p["lon"], p["lat"]) for p in el["geometry"]]

        if el["type"] == "way" and "highway" in el.get("tags", {}):
            geom = LineString(coords)

        elif el["type"] == "way" and "building" in el.get("tags", {}):
            geom = Polygon(coords)

        else:
            continue

        features.append({
            "type": "Feature",
            "geometry": mapping(geom),
            "properties": el.get("tags", {})
        })

    return {"type": "FeatureCollection", "features": features}

bbox = (51.50, -0.15, 51.52, -0.10)
data = fetch_osm(bbox)

geojson = osm_to_geojson(data)

print(json.dumps(geojson, indent=2))