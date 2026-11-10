"""A bounded Python-syntax expression interpreter, NOT eval/exec or arbitrary Python."""
import ast
import json
import math
import operator
import sys

def number(value):
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > 1e100:
        raise ValueError('有限数字超出限制')
    return value

def series(value):
    if not isinstance(value, list) or not value or len(value) > 10000:
        raise ValueError('需要 1..10000 个样本')
    return [number(x) for x in value]

def interpret(code, context):
    tree = ast.parse(code, mode='eval')
    if sum(1 for _ in ast.walk(tree)) > 200:
        raise ValueError('表达式过于复杂')
    funcs = {'mean': lambda x: sum(series(x))/len(x),
             'rms': lambda x: math.sqrt(sum(v*v for v in series(x))/len(x)),
             'min': lambda x: min(series(x)), 'max': lambda x: max(series(x)),
             'sum': lambda x: sum(series(x)), 'len': lambda x: len(series(x)),
             'abs': lambda x: abs(number(x)), 'sqrt': lambda x: math.sqrt(number(x))}
    ops = {ast.Add: operator.add, ast.Sub: operator.sub,
           ast.Mult: operator.mul, ast.Div: operator.truediv}
    def visit(n, depth=0):
        if depth > 20:
            raise ValueError('嵌套超过 20 层')
        def v(child):
            return visit(child, depth+1)
        if isinstance(n, ast.Expression):
            return v(n.body)
        if isinstance(n, ast.Constant) and type(n.value) in (int, float, str):
            return number(n.value) if not isinstance(n.value, str) else n.value
        if isinstance(n, ast.Name) and n.id in context:
            return context[n.id]
        if isinstance(n, ast.BinOp) and type(n.op) in ops:
            return number(ops[type(n.op)](number(v(n.left)), number(v(n.right))))
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.UAdd, ast.USub)):
            return number(v(n.operand)) * (-1 if isinstance(n.op, ast.USub) else 1)
        if isinstance(n, ast.Subscript):
            source, key = v(n.value), v(n.slice)
            if type(source) not in (dict, list) or type(key) not in (str, int):
                raise ValueError('非法索引')
            return source[key]
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id in funcs and len(n.args)==1 and not n.keywords:
            return number(funcs[n.func.id](v(n.args[0])))
        raise ValueError('不允许该语法；仅支持只读数值表达式')
    result = visit(tree)
    if isinstance(result, (dict, list)):
        raise ValueError('结果必须为标量，请选择字段或统计函数')
    return result

def main():
    raw = sys.stdin.buffer.read(2*1024*1024+1)
    if len(raw)>2*1024*1024:
        raise ValueError('输入过大')
    data = json.loads(raw)
    for line in data.pop('code').splitlines()[:100]:
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        try:
            print(json.dumps({'expression':line,'result':interpret(line,data)},ensure_ascii=False))
        except Exception as exc:
            print(json.dumps({'expression':line,'error':str(exc)},ensure_ascii=False))
if __name__=='__main__':
    main()
