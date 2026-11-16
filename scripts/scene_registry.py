"""Scene registry for additive old/new map and visual-bank selection."""
import json
from pathlib import Path

def load_scenes(root):
    path=Path(root)/"config"/"scenes.json"
    data=json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema")!=1 or not isinstance(data.get("scenes"),dict):
        raise ValueError("invalid config/scenes.json")
    return data

def scene_bundle(root,name=None):
    root=Path(root); data=load_scenes(root); name=name or data.get("default","old")
    if name not in data["scenes"]:
        raise ValueError("unknown scene: "+str(name))
    spec=dict(data["scenes"][name]); spec["name"]=name; spec["root"]=root
    spec["visual_map"]=root/spec["visual_map"]
    spec["maps"]={floor:{"yaml":root/item["yaml"],"image":root/item["image"]} for floor,item in spec["maps"].items()}
    if not spec["visual_map"].is_file(): raise ValueError("missing visual map for scene "+name)
    for floor,item in spec["maps"].items():
        if not item["yaml"].is_file() or not item["image"].is_file(): raise ValueError("missing "+floor+" map for scene "+name)
    return spec

def scene_list(root):
    data=load_scenes(root)
    return [{"name":name,"label":spec.get("label",name),"map_id":spec["map_id"],"navigation_enabled":bool(spec.get("navigation_enabled",False)),"note":spec.get("note","")} for name,spec in data["scenes"].items()]
