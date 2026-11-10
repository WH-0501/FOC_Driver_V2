import importlib.util
import json
import pathlib
import subprocess
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('worker', ROOT/'compat/expression_worker.py')
worker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(worker)

class Expressions(unittest.TestCase):
    def test_statistics(self):
        self.assertEqual(worker.interpret('mean(iq)', {'iq':[1,2,3]}), 2)
        self.assertEqual(worker.interpret("telemetry['node']", {'telemetry':{'node':7}}),7)

    def test_no_privilege_escape(self):
        for expression in ("__import__('os')", "iq.__class__", "open('x')", "[x for x in iq]",
                           "(lambda: 1)()", "'x' * 100000000", "2 ** 100000000", "10 ** 10"):
            with self.subTest(expression=expression), self.assertRaises((ValueError, SyntaxError)):
                worker.interpret(expression, {'iq':[1,2]})

    def test_embedded_process(self):
        executable=ROOT/'compat/python/python.exe'
        if not executable.exists():
            self.skipTest('Windows embedded runtime not deployed')
        proc=subprocess.run([str(executable),'-I',str(ROOT/'compat/expression_worker.py')],
                            input=json.dumps({'code':'mean(iq)','iq':[2,4]}).encode(),
                            stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=5)
        self.assertEqual(proc.returncode,0,proc.stderr)
        self.assertEqual(json.loads(proc.stdout)['result'],3)

    def test_virtual_backend_process(self):
        executable=ROOT/'compat/python/python.exe'
        if not executable.exists():
            self.skipTest('Windows embedded runtime not deployed')
        lines=[{'op':'open','profile':'Virtual CAN-FD (软件测试)','channel':'motortune-regression'},
               {'op':'send','node':1,'data':'014100'}, {'op':'close'}]
        proc=subprocess.run([str(executable),'-u',str(ROOT/'compat/bridge.py')],
                            input=('\n'.join(json.dumps(x) for x in lines)+'\n').encode(),
                            stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=10)
        self.assertEqual(proc.returncode,0,proc.stderr)
        results=[json.loads(x) for x in proc.stdout.splitlines()]
        self.assertTrue(any(x.get('opened') for x in results),results)
        self.assertFalse(any('error' in x for x in results),results)

if __name__=='__main__':
    unittest.main()
