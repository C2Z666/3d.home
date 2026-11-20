import signal,subprocess,sys,tempfile,threading,unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from web_navigation import NavigationManager


class WalkResetManagerTests(unittest.TestCase):
    def test_old_task_is_reaped_before_reset_and_preview_is_invalidated(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'scripts').mkdir()
            (root/'scripts/run_walk_reset.sh').write_text('#!/bin/bash\necho "[WALK_RESET] confirmed offline fixture"\n')
            manager=NavigationManager(root,SimpleNamespace(scene='old'))
            old=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
            manager.process=old;manager.task=dict(task_id='old',state='RUNNING',max_runtime=30,reason='test')
            manager.plans['old']={}
            manager.monitor_thread=threading.Thread(target=manager._monitor,args=(old,'old'),daemon=True);manager.monitor_thread.start()
            try:
                with self.assertRaises(ValueError):manager.reset_walk({})
                self.assertIsNone(old.poll())
                result=manager.reset_walk({'confirmed':True})
                self.assertIsNotNone(old.poll());self.assertFalse(manager.plans)
                self.assertEqual(result['task']['kind'],'walk_reset')
                manager.process.wait(timeout=5);manager.monitor_thread.join(timeout=5)
                self.assertEqual(manager.task['state'],'SUCCEEDED')
                self.assertNotEqual(manager.task['task_id'],'old')
            finally:
                manager.close()


if __name__=='__main__':unittest.main()
