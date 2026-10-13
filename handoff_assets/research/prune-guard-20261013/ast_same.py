"""Same syntax tree once docstrings are removed? argv: a.py b.py"""
import ast, sys
def tree(p):
    t = ast.parse(open(p, encoding="utf-8").read())
    for node in ast.walk(t):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and node.body \
                and isinstance(node.body[0], ast.Expr) and isinstance(getattr(node.body[0], "value", None), ast.Constant) \
                and isinstance(node.body[0].value.value, str):
            node.body = node.body[1:]
    return ast.dump(t)
print("same syntax tree without docstrings:", tree(sys.argv[1]) == tree(sys.argv[2]))
