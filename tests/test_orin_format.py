import ast, json, re, sys, unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from cloud_scan import project
from site_identity import verify

class OrinFormatTests(unittest.TestCase):
    def test_linux_text_format(self):
        extensions={'.py','.sh','.bash','.md','.html','.yaml','.yml','.json','.xml','.cpp','.hpp','.h','.txt'}
        for p in ROOT.rglob('*'):
            if not p.is_file() or any(x in {'build','install','log','work','__pycache__'} for x in p.relative_to(ROOT).parts):continue
            if p.suffix not in extensions:continue
            with self.subTest(path=str(p.relative_to(ROOT))):
                b=p.read_bytes();t=b.decode('utf-8')
                self.assertFalse(b.startswith(b'\xef\xbb\xbf'))
                self.assertNotIn(b'\r',b)
                self.assertNotIn('\ufffd',t)
                self.assertFalse(any(0xe000<=ord(c)<=0xf8ff for c in t))
    def test_scripts_use_own_environment(self):
        for p in (ROOT/'scripts').glob('*.sh'):
            t=p.read_text(encoding='utf-8')
            self.assertNotIn('/home/robot/zhangzhang/d1_stair_nav_phase3/scripts/env_live.sh',t)
        self.assertIn('PYTHONNOUSERSITE=1',(ROOT/'scripts/env_live.sh').read_text(encoding='utf-8'))
    def test_map_identity_and_scan_fix_are_retained(self):
        self.assertTrue(verify(ROOT).startswith('retained_'))
        tree=ast.parse((ROOT/'scripts/cloud_scan.py').read_text(encoding='utf-8'))
        call=next(n.value for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='ranges' for t in n.targets))
        code=compile(ast.Expression(call),'online_scan_call','eval')
        cfg=dict(range_min_m=.3,range_max_m=8.,base_height_m=.47,obstacle_min_above_floor_m=.12,obstacle_max_above_floor_m=1.2)
        # Identity extrinsics preserve the raw axes. atan2(+Y, +X) sets scan bins.
        for point,expected in [([2.,0,0],360),([0,2.,0],540),([-2.,0,0],0),([0,-2.,0],180)]:
            with self.subTest(point=point):
                out=eval(code,dict(project=project,xyz=np.array([point]),N_B=np.eye(4),B_L=np.eye(4),c=cfg))
                self.assertEqual(np.flatnonzero(np.isfinite(out)).tolist(),[expected])
                self.assertEqual(out[expected],2.)
    def test_web_text_and_ids(self):
        from html.parser import HTMLParser
        class Elements(HTMLParser):
            def __init__(self):super().__init__();self.ids=[]
            def handle_starttag(self,tag,attrs):
                for k,v in attrs:
                    if k=='id':self.ids.append(v)
        t=(ROOT/'web/index.html').read_text(encoding='utf-8');parser=Elements();parser.feed(t)
        self.assertEqual(len(parser.ids),len(set(parser.ids)))
        for ident in set(re.findall(r"\$\('([^']+)'\)",t)):
            self.assertIn(ident,parser.ids)
        self.assertIn('<title>D1 地图与融合定位</title>',t)
        self.assertIn('已保存参考点',t)

if __name__=='__main__':unittest.main()
