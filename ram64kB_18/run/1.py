#!/usr/bin/env python3
"""Netlist Studio (Tk) - structural Verilog netlist viewer and ECO editor.

Single file, Python standard library only (needs python3-tk).
Run:  python3 netlist_studio.py [-lib a.lib b.lib ...] [-netlist a.v
b.v ...] [-sample] [-threads N]
      (-threads N: worker processes for heavy steps, 0 = all cores;
the double-dash forms --lib/--netlist/--sample/--threads also work)
"""
import weakref
import glob, signal
import re, os, sys, copy, json, time, zlib, textwrap, shutil, subprocess, tempfile, threading, queue, math, bisect, pickle, traceback
from array import array

# ====================================================================
#  Liberty (NLDM / CCS / ECSM aware)
# ====================================================================
LIB_KEEP = {'library', 'cell', 'pin', 'bus', 'bundle', 'ff', 'latch',
'ff_bank', 'latch_bank',
            'statetable', 'type', 'memory', 'pg_pin'}
NLDM_KEYS = ('cell_rise', 'cell_fall', 'rise_transition',
'fall_transition', 'rise_constraint', 'fall_constraint')
CCS_KEYS = ('output_current_rise', 'output_current_fall',
'receiver_capacitance1_rise',
            'receiver_capacitance1_fall',
'receiver_capacitance2_rise', 'receiver_capacitance2_fall')

class GNode:
    __slots__ = ('type', 'args', 'attrs', 'children', 'start', 'end')
    def __init__(self, type_, args):
        self.type = type_; self.args = args; self.attrs = {};
self.children = []; self.start = 0; self.end = 0

_BR = re.compile(r'[{}"]')
_PAR = re.compile(r'[()"]')
_WS = re.compile(r'[\s;]*')
_WS2 = re.compile(r'\s*')
_ID = re.compile(r'([^\s:(){};,"]+)\s*')

def _skip_group(t, i):
    d = 1
    n = len(t)
    while True:
        m = _BR.search(t, i)
        if not m:
            return n
        c = m.group()
        if c == '"':
            j = t.find('"', m.end())
            i = n if j < 0 else j + 1
        elif c == '{':
            d += 1; i = m.end()
        else:
            d -= 1; i = m.end()
            if d == 0:
                return i

def _read_paren(t, i):
    d = 1; s = i + 1; i = s; n = len(t)
    while True:
        m = _PAR.search(t, i)
        if not m:
            return t[s:], n
        c = m.group()
        if c == '"':
            j = t.find('"', m.end())
            i = n if j < 0 else j + 1
        elif c == '(':
            d += 1; i = m.end()
        else:
            d -= 1; i = m.end()
            if d == 0:
                return t[s:m.start()], i

def _read_value(t, i):
    i = _WS2.match(t, i).end()
    if t[i:i + 1] == '"':
        j = t.find('"', i + 1)
        if j < 0: j = len(t)
        v = t[i + 1:j]
        k = t.find(';', j)
        return v, (len(t) if k < 0 else k + 1)
    k = t.find(';', i)
    if k < 0: k = len(t)
    return t[i:k].strip().strip('"'), k + 1

def split_args(raw):
    out = []; cur = []; q = False
    for ch in raw:
        if ch == '"': q = not q; continue
        if ch == ',' and not q:
            out.append(''.join(cur).strip()); cur = []
        elif ch == '\\' and False:
            pass
        else:
            cur.append(ch)
    out.append(''.join(cur).strip())
    return [x for x in out if x != '']

def _parse_body(t, i, node):
    n = len(t)
    while True:
        i = _WS.match(t, i).end()
        if i >= n: return n
        if t[i] == '}': return i + 1
        m = _ID.match(t, i)
        if not m:
            i += 1; continue
        name = m.group(1); i = m.end()
        c = t[i:i + 1]
        if c == ':':
            v, i = _read_value(t, i + 1)
            node.attrs[name] = v
        elif c == '(':
            args, i = _read_paren(t, i)
            j = _WS2.match(t, i).end()
            if t[j:j + 1] == '{':
                j += 1
                if name in LIB_KEEP:
                    ch = GNode(name, split_args(args)); ch.start = j
                    node.children.append(ch)
                    i = _parse_body(t, j, ch); ch.end = i
                else:
                    i = _skip_group(t, j)
            else:
                if t[j:j + 1] == ';': j += 1
                node.attrs[name] = args; i = j
        else:
            i += 1

class Pin:
    __slots__ = ('name', 'dir', 'func', 'tri', 'cap', 'clock',
'width', 'msb', 'lsb', 'is_bus', 'attrs')

class Cell:
    def __init__(self, name):
        self.name = name; self.area = 0.0; self.attrs = {}; self.pins = []; self.pin_map = {}; self.pg = []
        self.ff = None; self.latch = None; self.statetable = None; self.memory = False
        self.flags = {'nldm': 0, 'ccs': 0, 'ecsm': 0}; self.model = 'none'
        self._cls = None; self._sym = None; self._symb = None

class Lib:
    def __init__(self):
        self.name = ''; self.cells = {}; self.order = []; self.attrs = {}; self.types = {}
        self.stats = {'cells': 0, 'nldm': 0, 'ccs': 0, 'ecsm': 0, 'none': 0}; self.delay_model = ''

def parse_liberty(text):
    text = _CM_RE.sub('', text)
    text = text.replace('\\\r\n', ' ').replace('\\\n', ' ')
    root = GNode('root', [])
    _parse_body(text, 0, root)
    ln = None
    for ch in root.children:
        if ch.type == 'library': ln = ch; break
    if ln is None:
        raise ValueError('No library(...) group found in the Liberty text.')
    lib = Lib(); lib.name = ln.args[0] if ln.args else 'lib'; lib.attrs = ln.attrs
    lib.delay_model = ln.attrs.get('delay_model', '')
    for ch in ln.children:
        if ch.type == 'type':
            a = ch.attrs
            lib.types[ch.args[0]] = (_int(a.get('bit_from'), 0),
_int(a.get('bit_to'), 0))
    for ch in ln.children:
        if ch.type != 'cell': continue
        c = _build_cell(ch, lib.types, text)
        lib.cells[c.name] = c; lib.order.append(c.name)
        lib.stats['cells'] += 1
        for k in ('nldm', 'ccs', 'ecsm'):
            if c.flags[k]: lib.stats[k] += 1
        if not any(c.flags.values()): lib.stats['none'] += 1
    return lib

_CM_RE = re.compile(r'/\*.*?\*/', re.S)

def _int(v, d):
    try: return int(str(v).strip())
    except Exception: return d

def _build_cell(cn, types, text):
    c = Cell(cn.args[0] if cn.args else '')
    try: c.area = float(cn.attrs.get('area', 0))
    except ValueError: c.area = 0.0
    c.attrs = cn.attrs
    body = text[cn.start:cn.end]
    c.flags = {'nldm': sum(body.count(k) for k in NLDM_KEYS), 'ccs':
sum(body.count(k) for k in CCS_KEYS),
               'ecsm': body.count('ecsm_')}
    def add_pin(node, is_bus):
        for nm in (node.args or ['pin']):
            a = node.attrs
            p = Pin(); p.name = nm; p.dir = a.get('direction', 'input').lower(); p.func = a.get('function', '')
            p.tri = a.get('three_state', ''); p.clock = a.get('clock') == 'true'; p.width = 1; p.msb = 0; p.lsb = 0
            p.is_bus = is_bus; p.attrs = a
            try: p.cap = float(a.get('capacitance', 0))
            except ValueError: p.cap = 0.0
            if is_bus:
                t = types.get(a.get('bus_type', ''))
                if t:
                    p.msb, p.lsb = t; p.width = abs(t[0] - t[1]) + 1
            if p.dir == 'internal': continue
            c.pins.append(p); c.pin_map[nm] = p
    for ch in cn.children:
        if ch.type == 'pin': add_pin(ch, False)
        elif ch.type == 'bus': add_pin(ch, True)
        elif ch.type == 'bundle': add_pin(ch, False)
        elif ch.type == 'pg_pin': c.pg.append(ch.args[0] if ch.args else '')
        elif ch.type in ('ff', 'ff_bank'): c.ff = (ch.args, ch.attrs)
        elif ch.type in ('latch', 'latch_bank'): c.latch = (ch.args, ch.attrs)
        elif ch.type == 'statetable': c.statetable = (ch.args, ch.attrs)
        elif ch.type == 'memory': c.memory = True c.model = 'ECSM'
		  if c.flags['ecsm'] else 'CCS' 
		  if c.flags['ccs'] else 'NLDM' 
		  if c.flags['nldm'] else 'none'
        return c

# ---------------- boolean function expressions ----------------
_FTOK = re.compile(r"\s*([A-Za-z_][A-Za-z0-9_$\[\].]*|[01]|[!~'^*&+|()])")

def parse_func(s):
    """-> ('var',n) ('const',v) ('not',a) ('and'|'or'|'xor',[...]) or None"""
    if s is None: return None
    s = str(s).strip()
    if not s: return None
    toks = []; pos = 0
    while pos < len(s):
        m = _FTOK.match(s, pos)
        if not m:
            if s[pos:].strip() == '': break
            return None
        toks.append(m.group(1)); pos = m.end()
    st = [0]
    def peek(): return toks[st[0]] if st[0] < len(toks) else None
    def operand_start(t): return t is not None and (t in ('(', '!',
'~') or re.match(r'[A-Za-z_01]', t))
    def flat(op, a):
        r = []
        for x in a:
            if x[0] == op: r.extend(x[1])
            else: r.append(x)
        return r
    def p_or():
        a = [p_and()]
        while peek() in ('+', '|'):
            st[0] += 1; a.append(p_and())
        return a[0] if len(a) == 1 else ('or', flat('or', a))
    def p_and():
        a = [p_xor()]
        while True:
            t = peek()
            if t in ('*', '&'):
                st[0] += 1; a.append(p_xor())
            elif operand_start(t):
                a.append(p_xor())
            else: break
        return a[0] if len(a) == 1 else ('and', flat('and', a))
    def p_xor():
        a = [p_un()]
        while peek() == '^':
            st[0] += 1; a.append(p_un())
        return a[0] if len(a) == 1 else ('xor', flat('xor', a))
    def p_un():
        t = peek()
        if t in ('!', '~'):
            st[0] += 1; x = ('not', p_un())
        else:
            x = p_prim()
        while peek() == "'":
            st[0] += 1; x = ('not', x)
        return x
    def p_prim():
        if st[0] >= len(toks): raise ValueError
        t = toks[st[0]]; st[0] += 1
        if t == '(':
            x = p_or()
            if peek() != ')': raise ValueError
            st[0] += 1; return x
        if t == '0': return ('const', 0)
        if t == '1': return ('const', 1)
        if re.match(r'[A-Za-z_]', t): return ('var', t)
        raise ValueError
    try:
        ast = p_or()
        if st[0] < len(toks): return None
        return ast
    except (ValueError, IndexError):
        return None

def ast_vars(a, acc=None):
    if acc is None: acc = []
    if a is None: return acc
    k = a[0]
    if k == 'var':
        if a[1] not in acc: acc.append(a[1])
    elif k == 'not': ast_vars(a[1], acc)
    elif k in ('and', 'or', 'xor'):
        for x in a[1]: ast_vars(x, acc)
    return acc

def ast_eval(a, env):
    k = a[0]
    if k == 'const': return a[1]
    if k == 'var': return 1 if env.get(a[1]) else 0
    if k == 'not': return 0 if ast_eval(a[1], env) else 1
    if k == 'and': return 1 if all(ast_eval(x, env) for x in a[1]) else 0
    if k == 'or': return 1 if any(ast_eval(x, env) for x in a[1]) else 0
    s = 0
    for x in a[1]: s ^= ast_eval(x, env)
    return s

def truth_table(a, vars_):
    n = len(vars_); tt = []
    for r in range(1 << n):
        env = {vars_[i]: (r >> i) & 1 for i in range(n)}
        tt.append(ast_eval(a, env))
    return tt

def ast_to_verilog(a):
    k = a[0]
    if k == 'const': return "1'b%d" % a[1]
    if k == 'var': return a[1]
    if k == 'not':
        x = a[1]
        return '~' + (ast_to_verilog(x) if x[0] in ('var', 'const')
else '(' + ast_to_verilog(x) + ')')
    op = {'and': ' & ', 'or': ' | ', 'xor': ' ^ '}[k]
    return '(' + op.join(ast_to_verilog(x) for x in a[1]) + ')'

# ====================================================================
#  Function-driven cell classification
# ====================================================================
def is_lit(a): return a[0] == 'var' or (a[0] == 'not' and a[1][0] == 'var')
def lit_pin(a): return (a[1], False) if a[0] == 'var' else (a[1][1],
True)   # (name, inverted)
def lit_of(a): return lit_pin(a) if a and is_lit(a) else None

def gate_from_ast(ast, out):
    inv = False; a = ast
    while a[0] == 'not': inv = not inv; a = a[1]
    if a[0] == 'var':
        return {'kind': 'gate', 'gate': 'buf', 'inv': inv, 'inputs':
[[a[1], False]], 'out': out}
    if a[0] not in ('and', 'or', 'xor'): return None
    if not all(is_lit(x) for x in a[1]): return None
    leaves = [list(lit_pin(x)) for x in a[1]]; op = a[0]
    names = [l[0] for l in leaves]
    if len(set(names)) != len(names): return None
    if op == 'xor':
        if sum(1 for l in leaves if l[1]) % 2: inv = not inv
        for l in leaves: l[1] = False
    elif all(l[1] for l in leaves):
        for l in leaves: l[1] = False
        op = 'or' if op == 'and' else 'and'; inv = not inv
    return {'kind': 'gate', 'gate': op, 'inv': inv, 'inputs': leaves,
'out': out}

def gate_name(d):
    if d['gate'] == 'buf': return 'INV' if d['inv'] else 'BUF'
    base = {'and': 'AND', 'or': 'OR', 'xor': 'XOR'}[d['gate']]
    if d['inv']: base = {'and': 'NAND', 'or': 'NOR', 'xor': 'XNOR'}[d['gate']]
    return base + str(len(d['inputs']))

def detect_mux(ast):
    vars_ = ast_vars(ast); n = len(vars_)
    k = 1 if n == 3 else 2 if n == 6 else 0
    if not k: return None
    tt = truth_table(ast, vars_)
    combos = [[i] for i in range(3)] if k == 1 else [[i, j] for i in
range(6) for j in range(6) if i != j]
    for sel in combos:
        data = [i for i in range(n) if i not in sel]
        for inv in (0, 1):
            mp = []; ok = True; used = set()
            for v in range(1 << k):
                found = -1
                for d in data:
                    if d in used: continue
                    good = True
                    for r in range(1 << n):
                        sv = 0
                        for s_i, s in enumerate(sel): sv |= ((r >> s)
& 1) << s_i
                        if sv != v: continue
                        if (tt[r] ^ inv) != ((r >> d) & 1): good = False; break
                    if good: found = d; break
                if found < 0: ok = False; break
                mp.append(vars_[found]); used.add(found)
            if ok:
                sn = [vars_[x] for x in sel]
                if len(sn) == 2 and sn[0] > sn[1]:
                    sn = [sn[1], sn[0]]; mp = [mp[0], mp[2], mp[1], mp[3]]
                return {'kind': 'mux', 'sel': sn, 'data': mp, 'inv': bool(inv)}
    return None

def cplx_from_ast(ast, out):
    inv = False; a = ast
    while a[0] == 'not': inv = not inv; a = a[1]
    if a[0] not in ('or', 'and'): return None
    inner = 'and' if a[0] == 'or' else 'or'; terms = []
    for ch in a[1]:
        if is_lit(ch): terms.append([lit_pin(ch)])
        elif ch[0] == inner and all(is_lit(x) for x in ch[1]):
terms.append([lit_pin(x) for x in ch[1]])
        else: return None
    if not any(len(t) > 1 for t in terms): return None
    names = [l[0] for t in terms for l in t]
    if len(set(names)) != len(names): return None
    sizes = ''.join(str(x) for x in sorted((len(t) for t in terms),
reverse=True))
    nm = ('AOI' if inv else 'AO') + sizes if a[0] == 'or' else ('OAI'
if inv else 'OA') + sizes
    return {'kind': 'cplx', 'outer': a[0], 'inv': inv, 'terms': terms,
'out': out, 'label': nm}

def input_pins(c): return [p for p in c.pins if p.dir == 'input']
def output_pins(c): return [p for p in c.pins if p.dir in ('output', 'inout')]

def classify_cell(cell):
    if cell._cls is not None: return cell._cls
    try: c = _do_classify(cell)
    except Exception: c = None
    if c is None: c = {'kind': 'box', 'title': '', 'cat': 'other'}
    cell._cls = c
    return c

def _do_classify(cell):
    ins = input_pins(cell); outs = output_pins(cell)
    if cell.attrs.get('clock_gating_integrated_cell'): return {'kind':
'icg', 'cat': 'seq'}
    if cell.ff or cell.latch: return classify_seq(cell)
    if cell.memory: return {'kind': 'box', 'title': 'MEM', 'cat': 'macro'}
    if cell.statetable: return {'kind': 'box', 'title': 'STATE', 'cat': 'seq'}
    if not outs: return {'kind': 'box', 'title': '', 'cat': 'other'}
    if len(outs) > 1: return classify_multi(cell, outs, ins)
    o = outs[0]
    if not o.func: return {'kind': 'box', 'title': '', 'cat': 'macro'}
    ast = parse_func(o.func)
    if ast is None: return {'kind': 'box', 'title': '?', 'cat':
'other', 'note': o.func}
    vs = ast_vars(ast); in_names = [p.name for p in ins]
    covers = len(ins) == len(vs) and all(v in in_names for v in vs)
    if o.tri:
        g0 = gate_from_ast(ast, o.name); tl = lit_of(parse_func(o.tri))
        if g0 and g0['gate'] == 'buf' and tl and len(ins) == 2:
            return {'kind': 'tbuf', 'inv': g0['inv'], 'input':
g0['inputs'][0][0], 'en': (tl[0], not tl[1]), 'out': o.name, 'cat':
'comb'}
        return {'kind': 'box', 'title': 'TRI', 'cat': 'comb', 'note': o.func}
    if ast[0] == 'const': return {'kind': 'tie', 'val': ast[1], 'out':
o.name, 'cat': 'comb'}
    if not covers: return {'kind': 'box', 'title': '', 'cat': 'comb',
'note': o.func}
    g = gate_from_ast(ast, o.name)
    if g: g['cat'] = 'comb'; g['label'] = gate_name(g); return g
    m = detect_mux(ast)
    if m: m['out'] = o.name; m['cat'] = 'comb'; m['label'] = ('IMUX'
if m['inv'] else 'MUX') + str(1 << len(m['sel'])); return m
    cp = cplx_from_ast(ast, o.name)
    if cp: cp['cat'] = 'comb'; return cp
    return {'kind': 'box', 'title': '', 'cat': 'comb', 'note': o.func}

def classify_multi(cell, outs, ins):
    asts = [parse_func(o.func) for o in outs]
    if all(a is not None for a in asts):
        if all(a[0] == 'const' for a in asts): return {'kind': 'box',
'title': 'TIE', 'cat': 'comb'}
        vs = []
        for a in asts: ast_vars(a, vs)
        n = len(vs)
        if len(outs) == 2 and n in (2, 3) and len(ins) == n:
            tts = [truth_table(a, vs) for a in asts]; rows = 1 << n;
par = []; maj = []
            for r in range(rows):
                c = bin(r).count('1'); par.append(c & 1);
maj.append((1 if c == 2 else 0) if n == 2 else (1 if c >= 2 else 0))
            for s in (0, 1):
                for is_ in (0, 1):
                    for ic in (0, 1):
                        ts, tc = tts[s], tts[1 - s]
                        if all(ts[r] == par[r] ^ is_ and tc[r] ==
maj[r] ^ ic for r in range(rows)):
                            return {'kind': 'adder', 'type': 'ha' if n
== 2 else 'fa', 'sum': (outs[s].name, bool(is_)),
                                    'carry': (outs[1 - s].name,
bool(ic)), 'cat': 'comb'}
    return {'kind': 'box', 'title': '', 'cat': 'comb', 'note': '%d
outputs' % len(outs)}

def classify_seq(cell):
    ff = cell.ff is not None
    args, A = cell.ff if ff else cell.latch
    iq = args[0]; iqn = args[1] if len(args) > 1 else None
    d = {'kind': 'seq', 'type': 'dff' if ff else 'latch', 'cat':
'seq', 'data': [], 'en': None, 'scan': None,
         'clk': None, 'clr': None, 'pre': None, 'outs': [], 'extra': []}
    cl = lit_of(parse_func(A.get('clocked_on') if ff else A.get('enable')))
    if cl: d['clk'] = (cl[0], cl[1])
    cr = lit_of(parse_func(A.get('clear'))) if A.get('clear') else None
    pr = lit_of(parse_func(A.get('preset'))) if A.get('preset') else None
    if cr: d['clr'] = (cr[0], cr[1])
    if pr: d['pre'] = (pr[0], pr[1])
    ns = parse_func(A.get('next_state') if ff else A.get('data_in'))
    if ns:
        nsl = lit_of(ns)
        if nsl: d['data'] = [nsl[0]]
        else:
            mx = detect_mux(ns)
            if mx and len(mx['sel']) == 1 and not mx['inv']:
                s = mx['sel'][0]; d0, d1 = mx['data']
                if d0 in (iq, iqn): d['en'] = (s, False); d['data'] = [d1]
                elif d1 in (iq, iqn): d['en'] = (s, True); d['data'] = [d0]
                else: d['scan'] = (s, d1); d['data'] = [d0]
            else:
                d['data'] = [v for v in ast_vars(ns) if v not in (iq, iqn)]
    used = set()
    for x in (d['clk'], d['clr'], d['pre']):
        if x: used.add(x[0])
    if d['en']: used.add(d['en'][0])
    if d['scan']: used.update(d['scan'])
    used.update(d['data'])
    for p in output_pins(cell):
        a = parse_func(p.func); inv = False; name = None
        if a:
            x = a
            while x[0] == 'not': inv = not inv; x = x[1]
            if x[0] == 'var':
                if x[1] == iq: name = iq
                elif x[1] == iqn: name = iqn
        d['outs'].append((p.name, inv if name else False))
    for p in input_pins(cell):
        if p.name not in used: d['extra'].append(p.name)
    return d

def describe_cell(cell):
    d = classify_cell(cell); k = d['kind']
    if k == 'gate':
        if d['gate'] == 'buf': return 'Inverter' if d['inv'] else 'Buffer'
        return d['label'] + ' gate' + (' with inverted inputs' if
any(l[1] for l in d['inputs']) else '')
    if k == 'mux': return d['label'] + ' multiplexer'
    if k == 'cplx': return d['label'] + ' complex gate'
    if k == 'tie': return 'Tie-' + ('high' if d['val'] else 'low') + ' cell'
    if k == 'tbuf': return 'Tri-state ' + ('inverter' if d['inv'] else 'buffer')
    if k == 'adder': return 'Full adder' if d['type'] == 'fa' else 'Half adder'
    if k == 'icg': return 'Integrated clock gate'
    if k == 'seq':
        if d['type'] == 'latch':
            s = 'Latch (transparent ' + ('low' if d['clk'] and
d['clk'][1] else 'high') + ')'
        else:
            s = ('Negative' if d['clk'] and d['clk'][1] else
'Positive') + '-edge D flip-flop'
        if d['en']: s += ' with enable'
        if d['scan']: s += ' with scan'
        if d['clr']: s += ', async clear'
        if d['pre']: s += ', async preset'
        return s
    t = d.get('title')
    if t == 'MEM': return 'Memory / macro'
    if t == 'TIE': return 'Tie cell (constant outputs)'
    if t == 'STATE': return 'State-table cell'
    return 'Custom function' if d.get('note') else 'Generic block'

# ====================================================================
#  Symbols: drawing primitives generated from the classification
# ====================================================================
# primitives: ('L',pts,dash) ('P',pts,fill,dash) ('O',cx,cy,r,fill)
('T',x,y,text,size,anchor,style)
class Sym:
    __slots__ = ('w', 'h', 'pins', 'prims', 'label', 'box', 'port')
    def __init__(self, w, h, pins, prims, label='', box=False, port=False):
        self.w = w; self.h = h; self.pins = pins; self.prims = prims;
self.label = label; self.box = box; self.port = port

def _pin(x, y, side, dir_, **kw):
    d = {'x': x, 'y': y, 'side': side, 'dir': dir_, 'low': False,
'wedge': False, 'inv': False, 'label': '', 'width': 1}
    d.update(kw); return d

def _bez(p0, p1, p2, n=10):
    out = []
    for i in range(n + 1):
        t = i / n; u = 1 - t
        out.append((u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0],
u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1]))
    return out

def _and_body(x0, xe, top, bot):
    xm = x0 + (xe - x0) * 0.45; cy = (top + bot) / 2; rx = xe - xm; ry
= (bot - top) / 2
    pts = [(x0, top), (xm, top)]
    for i in range(1, 12):
        a = -math.pi / 2 + math.pi * i / 12
        pts.append((xm + rx * math.cos(a), cy + ry * math.sin(a)))
    pts += [(xm, bot), (x0, bot)]
    return pts

def _or_body(x0, xe, top, bot):
    bw = xe - x0; cy = (top + bot) / 2
    return (_bez((x0, top), (x0 + bw * .55, top), (xe, cy)) +
_bez((xe, cy), (x0 + bw * .55, bot), (x0, bot))[1:] +
            _bez((x0, bot), (x0 + bw * .3, cy), (x0, top))[1:])

def _body(gate, x0, xe, top, bot):
    return _and_body(x0, xe, top, bot) if gate == 'and' else
_or_body(x0, xe, top, bot)

def gate_sym(d):
    ins = d['inputs']; n = len(ins); is_buf = d['gate'] == 'buf'
    h = 32 if is_buf else max(32, n * 16); w = 66; bub = 8 if d['inv'] else 0
    x0 = 12; xe = w - 12 - bub; cy = h / 2; top = 1; bot = h - 1
    bx0 = x0 + (5 if d['gate'] == 'xor' else 0)
    pins = {}; prims = []
    if is_buf: prims.append(('P', [(x0, top), (xe, cy), (x0, bot)],
True, False))
    else: prims.append(('P', _body(d['gate'], bx0, xe, top, bot), True, False))
    if d['gate'] == 'xor': prims.append(('L', _bez((x0, top), (x0 +
0.3 * (xe - bx0), cy), (x0, bot)), False))
    for i, (nm, low) in enumerate(ins):
        y = cy if is_buf else (i + 0.5) * h / n
        t = (y - top) / (bot - top); edge = bx0
        if d['gate'] in ('or', 'xor'): edge = bx0 + 0.6 * (xe - bx0) *
t * (1 - t) - (5 if d['gate'] == 'xor' else 0)
        pins[nm] = _pin(0, y, 'L', 'in', low=low)
        prims.append(('L', [(0, y), (edge - (8 if low else 0), y)], False))
        if low: prims.append(('O', edge - 4, y, 4, True))
    pins[d['out']] = _pin(w, cy, 'R', 'out')
    if d['inv']: prims.append(('O', xe + 4, cy, 4, True))
    prims.append(('L', [(xe + bub, cy), (w, cy)], False))
    return Sym(w, h, pins, prims, d.get('label') or gate_name(d))

def mux_sym(d):
    names = d['data'] + d['sel']; n = len(names); h = max(48, n * 16);
w = 60; x0 = 12; xe = w - 12; cy = h / 2
    pins = {}; prims = [('P', [(x0, 0), (xe, 10), (xe, h - 10), (x0,
h)], True, False)]
    for i, nm in enumerate(names):
        y = (i + 0.5) * h / n
        pins[nm] = _pin(0, y, 'L', 'in')
        prims.append(('L', [(0, y), (x0, y)], False))
        prims.append(('T', x0 + 3, y, str(i) if i < len(d['data'])
else 'S%d' % (i - len(d['data'])), 8, 'w', 'mute'))
    pins[d['out']] = _pin(w, cy, 'R', 'out')
    if d['inv']:
        prims.append(('O', xe + 4, cy, 4, True)); prims.append(('L',
[(xe + 8, cy), (w, cy)], False))
    else: prims.append(('L', [(xe, cy), (w, cy)], False))
    return Sym(w, h, pins, prims, d['label'])

def cplx_sym(d):
    pins = {}; prims = []; y = 8; x0 = 22; w = 76; bub = 8 if d['inv']
else 0; xe = w - 12 - bub
    term_ys = []
    for t in d['terms']:
        ys = []
        for l in t: ys.append(y); y += 16
        term_ys.append(ys); y += 6
    h = max(40, y - 6 + 8); cy = h / 2; top = 1; bot = h - 1
    prims.append(('P', _body(d['outer'], x0, xe, top, bot), True, False))
    xb = x0 - 9
    def edge_at(yy):
        if d['outer'] == 'and': return x0
        t = (yy - top) / (bot - top); return x0 + 0.6 * (xe - x0) * t * (1 - t)
    for t, ys in zip(d['terms'], term_ys):
        ymid = (ys[0] + ys[-1]) / 2
        for (nm, low), yy in zip(t, ys):
            pins[nm] = _pin(0, yy, 'L', 'in', low=low)
            if len(ys) == 1:
                e = edge_at(yy); prims.append(('L', [(0, yy), (e - (8
if low else 0), yy)], False))
                if low: prims.append(('O', e - 4, yy, 4, True))
            else:
                prims.append(('L', [(0, yy), (xb - (8 if low else 0),
yy)], False))
                if low: prims.append(('O', xb - 4, yy, 4, True))
        if len(ys) > 1:
            prims.append(('L', [(xb, ys[0]), (xb, ys[-1])], False));
prims.append(('L', [(xb, ymid), (edge_at(ymid), ymid)], False))
    prims.append(('T', (x0 + xe) / 2 + 2, cy, d['label'], 8, 'c', 'mute'))
    if d['inv']: prims.append(('O', xe + 4, cy, 4, True))
    prims.append(('L', [(xe + bub, cy), (w, cy)], False))
    pins[d['out']] = _pin(w, cy, 'R', 'out')
    return Sym(w, h, pins, prims, d['label'])

def tie_sym(d):
    w, h = 30, 24
    prims = [('P', [(0, 2), (22, 2), (22, 22), (0, 22)], True, False),
('T', 11, 12, str(d['val']), 11, 'c', 'bold'), ('L', [(22, 12), (w,
12)], False)]
    return Sym(w, h, {d['out']: _pin(w, 12, 'R', 'out')}, prims,
'TIEHI' if d['val'] else 'TIELO')

def tbuf_sym(d):
    w, h = 66, 48; x0 = 12; xe = w - 12 - (8 if d['inv'] else 0); cy = 24
    en, enlow = d['en']
    pins = {d['input']: _pin(0, cy, 'L', 'in'), en: _pin(0, 42, 'L',
'in', low=enlow), d['out']: _pin(w, cy, 'R', 'out')}
    xt = x0 + 14; yb = (h - 8) - ((h - 8) - cy) * (xt - x0) / (xe - x0)
    prims = [('P', [(x0, 2), (xe, cy), (x0, h - 8)], True, False),
('L', [(0, cy), (x0, cy)], False),
             ('L', [(0, 42), (xt, 42)], False), ('L', [(xt, 42), (xt,
yb)], False)]
    if enlow: prims.append(('O', xt - 9, 42, 3.5, True))
    if d['inv']: prims += [('O', xe + 4, cy, 4, True), ('L', [(xe + 8,
cy), (w, cy)], False)]
    else: prims.append(('L', [(xe, cy), (w, cy)], False))
    return Sym(w, h, pins, prims, 'TBUF')

CW = 6.0   # estimated character width at 9pt for box labels
def box_sym(title, left=(), right=(), sub='', note='', dashed=False,
double=False, min_w=88):
    left = list(left); right = list(right)
    rows = max(len(left), len(right), 1)
    lw = max([len(p.get('label') or p['name']) + (2 if p.get('low') or
p.get('wedge') else 0) for p in left] + [0])
    rw = max([len(p.get('label') or p['name']) + (2 if p.get('inv')
else 0) for p in right] + [0])
    tw = len(title) * 7 + 16; sw = len(sub) * 5.4 + 14; nt =
min(len(note), 34) * 5 + 14
    w = max(min_w, (lw + rw) * CW + 30, tw, sw, nt); w =
int(math.ceil(w / 4.0) * 4)
    top = 36 if (sub or note) else 26; h = top + rows * 16 + 4
    pins = {}; prims = [('P', [(0, 0), (w, 0), (w, h), (0, h)], True, dashed)]
    if double: prims.append(('P', [(3, 3), (w - 3, 3), (w - 3, h - 3),
(3, h - 3)], False, dashed))
    prims.append(('T', w / 2, 14, title, 10, 'c', 'bold'))
    if sub: prims.append(('T', w / 2, 27, sub, 8, 'c', 'mute'))
    if note: prims.append(('T', w / 2, 27, note if len(note) <= 34
else note[:33] + '...', 8, 'c', 'mute'))
    for i, p in enumerate(left):
        y = top + i * 16 + 4; low = bool(p.get('low')); lab =
p.get('label') or p['name']
        pins[p['name']] = _pin(-8 if low else 0, y, 'L', 'in',
low=low, wedge=bool(p.get('wedge')), label=lab, width=p.get('width',
1))
        if low: prims.append(('O', -4, y, 3.8, True))
        if p.get('wedge'): prims.append(('L', [(0, y - 5), (7, y), (0,
y + 5)], False))
        prims.append(('T', 9 if (low or p.get('wedge')) else 5, y,
lab, 8, 'w', 'mute'))
    for i, p in enumerate(right):
        y = top + i * 16 + 4; inv = bool(p.get('inv')); lab =
p.get('label') or p['name']
        pins[p['name']] = _pin(w + 8 if inv else w, y, 'R',
p.get('dir', 'out'), inv=inv, label=lab, width=p.get('width', 1))
        if inv: prims.append(('O', w + 4, y, 3.8, True))
        prims.append(('T', w - 5, y, lab, 8, 'e', 'mute'))
    return Sym(w, h, pins, prims, title, box=True)

def pin_label(p): return p.name + ('[%d:%d]' % (p.msb, p.lsb) if
p.is_bus or p.width > 1 else '')

def cell_box(cell, d=None):
    d = d or classify_cell(cell); L = []; R = []
    clk = d.get('clk')
    for p in cell.pins:
        lab = pin_label(p)
        if p.dir == 'input': L.append({'name': p.name, 'label': lab,
'width': p.width, 'wedge': p.clock or bool(clk and clk[0] == p.name)})
        else: R.append({'name': p.name, 'label': lab, 'width':
p.width, 'dir': 'io' if p.dir == 'inout' else 'out'})
    sub = d.get('label', '') if d['kind'] in ('gate', 'mux', 'cplx')
else d.get('title', '')
    return box_sym(cell.name, L, R, sub=sub, note=d.get('note', ''))

def build_cell_sym(cell):
    d = classify_cell(cell); k = d['kind']
    if k == 'gate':
        s = gate_sym(d)
        return s if len(s.pins) >= len(cell.pins) else cell_box(cell, d)
    if k == 'mux':
        s = mux_sym(d)
        return s if len(s.pins) >= len(cell.pins) else cell_box(cell, d)
    if k == 'cplx':
        s = cplx_sym(d)
        return s if len(s.pins) >= len(cell.pins) else cell_box(cell, d)
    if k == 'tie': return tie_sym(d)
    if k == 'tbuf': return tbuf_sym(d)
    if k == 'adder':
        L = [{'name': p.name} for p in cell.pins if p.dir == 'input']
        R = [{'name': p.name, 'inv': (d['sum'][0] == p.name and
d['sum'][1]) or (d['carry'][0] == p.name and d['carry'][1])}
             for p in cell.pins if p.dir != 'input']
        return box_sym(cell.name, L, R, sub='full adder' if d['type']
== 'fa' else 'half adder', min_w=96)
    if k == 'seq':
        Ls = []; seen = set()
        def addl(name, **kw):
            if not name or name in seen or name not in cell.pin_map: return
            seen.add(name); q = {'name': name, 'width':
cell.pin_map[name].width}; q.update(kw); Ls.append(q)
        for n in d['data']: addl(n)
        if d['scan']: addl(d['scan'][1]); addl(d['scan'][0])
        if d['en']: addl(d['en'][0], low=d['en'][1])
        for n in d['extra']: addl(n)
        if d['clk']: addl(d['clk'][0], wedge=d['type'] == 'dff',
low=d['clk'][1])
        if d['pre']: addl(d['pre'][0], low=d['pre'][1])
        if d['clr']: addl(d['clr'][0], low=d['clr'][1])
        for p in cell.pins:
            if p.dir == 'input': addl(p.name)
        Rs = [{'name': n, 'inv': inv, 'width': cell.pin_map[n].width
if n in cell.pin_map else 1} for n, inv in d['outs']]
        title = 'LATCH' if d['type'] == 'latch' else 'SDFF' if
d['scan'] else 'DFFE' if d['en'] else 'DFF'
        sub = (('transparent low' if d['clk'] and d['clk'][1] else
'transparent high') if d['type'] == 'latch'
               else ('negative edge' if d['clk'] and d['clk'][1] else
'positive edge'))
        return box_sym(cell.name, Ls, Rs, sub=title + ' · ' + sub, min_w=104)
    if k == 'icg':
        Li = [{'name': p.name, 'wedge':
p.attrs.get('clock_gate_clock_pin') == 'true' or p.clock} for p in
cell.pins if p.dir == 'input']
        Ri = [{'name': p.name} for p in cell.pins if p.dir != 'input']
        return box_sym(cell.name, Li, Ri, sub='clock gate (ICG)', min_w=100)
    return cell_box(cell, d)

def cell_symbol(cell, block=False):
    if block:
        if cell._symb is None: cell._symb = cell_box(cell)
        return cell._symb
    if cell._sym is None:
        try: cell._sym = build_cell_sym(cell)
        except Exception: cell._sym = cell_box(cell)
    return cell._sym

def port_sym(name, dir_, vec, msb, lsb):
    lab = name + ('[%d:%d]' % (msb, lsb) if vec and msb == msb else '')
    w = max(54, int(len(lab) * 6.6 + 26)); h = 26 if vec else 22; cy = h / 2
    if dir_ == 'input': pts = [(0, 0), (w - 9, 0), (w, cy), (w - 9,
h), (0, h)]; pin = _pin(w, cy, 'R', 'out')
    elif dir_ == 'output': pts = [(0, 0), (w - 9, 0), (w, cy), (w - 9,
h), (0, h)]; pin = _pin(0, cy, 'L', 'in')      # points away from the
logic: opposite to the flag on a wire's source side
    else: pts = [(9, 0), (w - 9, 0), (w, cy), (w - 9, h), (9, h), (0,
cy)]; pin = _pin(w, cy, 'R', 'io')
    prims = [('P', pts, True, False), ('T', 6, cy, lab, 9, 'w', 'bold')]
    return Sym(w, h, {'p': pin}, prims, lab, port=True)

def assign_sym():
    prims = [('P', [(6, 1), (28, 1), (28, 21), (6, 21)], True, False),
('T', 17, 11, '=', 12, 'c', 'bold'),
             ('L', [(0, 11), (6, 11)], False), ('L', [(28, 11), (34,
11)], False)]
    return Sym(34, 22, {'A': _pin(0, 11, 'L', 'in'), 'Y': _pin(34, 11,
'R', 'out')}, prims, 'assign')

PRIMS = {'and', 'nand', 'or', 'nor', 'xor', 'xnor', 'buf', 'not',
'bufif0', 'bufif1', 'notif0', 'notif1'}

def prim_pins(type_, nconn):
    t = type_.lower(); nin = max(1, nconn - 1)
    if t in ('buf', 'not'): nin = 1
    if t.endswith(('if0', 'if1')): return [('Y', 'output', 1), ('I1',
'input', 1), ('I2', 'input', 1)]
    return [('Y', 'output', 1)] + [('I%d' % i, 'input', 1) for i in
range(1, nin + 1)]

_PRIM_SYM = {}
def prim_symbol(type_, nconn, block=False):
    k = (type_, nconn, block); r = _PRIM_SYM.get(k)
    if r is None: r = _PRIM_SYM[k] = _prim_symbol(type_, nconn, block)
    return r
def _prim_symbol(type_, nconn, block=False):
    t = type_.lower(); nin = max(1, nconn - 1)
    if t in ('buf', 'not'): nin = 1
    if block or t.endswith(('if0', 'if1')):
        return box_sym(t.upper(), [{'name': n} for n, d, w in
prim_pins(type_, nconn) if d == 'input'], [{'name': 'Y'}],
sub='primitive', min_w=70)
    ins = [['I%d' % i, False] for i in range(1, nin + 1)]
    if t in ('buf', 'not'): d = {'gate': 'buf', 'inv': t == 'not'}
    else: d = {'gate': {'and': 'and', 'nand': 'and', 'or': 'or',
'nor': 'or', 'xor': 'xor', 'xnor': 'xor'}[t], 'inv': t in ('nand',
'nor', 'xnor')}
    d.update({'kind': 'gate', 'inputs': ins, 'out': 'Y', 'label': t.upper()})
    return gate_sym(d)

def module_symbol(mod):
    L = []; R = []
    for p in mod.ports:
        n = mod.nets.get(p.name); vec = bool(n and n.vec)
        lab = p.name + ('[%s:%s]' % (n.msb, n.lsb) if vec else '')
        wd = net_width(n) if n else 1
        if p.dir == 'input': L.append({'name': p.name, 'label': lab,
'width': wd})
        else: R.append({'name': p.name, 'label': lab, 'width': wd,
'dir': 'io' if p.dir == 'inout' else 'out'})
    return box_sym(mod.name, L, R, sub='module', double=True, min_w=110)

_UNK_SYM = {}
def unknown_symbol(inst):
    k = (inst.type, tuple(c.pin for c in inst.conns)); r = _UNK_SYM.get(k)
    if r is None:
        if len(_UNK_SYM) > 20000: _UNK_SYM.clear()
        r = _UNK_SYM[k] = _unknown_symbol(inst)
    return r
def _unknown_symbol(inst):
    L = []; R = []
    for i, c in enumerate(inst.conns):
        nm = c.pin or ('p%d' % i)
        if re.match(r'^(Z|ZN|Y|Q|QN|X|O|S|CO|SUM|OUT|GCLK|DO|DOUT|CON|SN)\d*$',
nm, re.I): R.append({'name': nm})
        else: L.append({'name': nm})
    return box_sym(inst.type, L, R, sub='undefined cell/module',
dashed=True, min_w=110)

# ====================================================================
#  Structural Verilog: model, parser, writer
# ====================================================================
T_ATTR, T_DIR, T_ESC, T_ID, T_NUM, T_STR, T_P = 2, 3, 4, 5, 6, 7, 8
NET_TYPES = {'wire', 'tri', 'wand', 'wor', 'supply0', 'supply1',
'tri0', 'tri1', 'trireg', 'reg', 'logic',
             'integer', 'real', 'time', 'genvar'}
_TOK = re.compile(r"(\s+|//[^\n]*|/\*.*?\*/)|(\(\*(?!\)).*?\*\))|(`[^\n]*)|(\\\S+)|([A-Za-z_][A-Za-z0-9_$]*)|"

r"(\d*'[sS]?[bBoOdDhH]\s*[0-9a-fA-F_xXzZ?]+|'[01xXzZ]|\d[\d_]*(?:\.\d+)?)|(\"(?:[^\"\\]|\\.)*\")|(.)",
re.S)

def lex(text, a=0, b=None):
    tt = []; tv = []; tp = []; te = []
    for m in _TOK.finditer(text, a, len(text) if b is None else b):
        g = m.lastindex
        if g == 1: continue
        tt.append(g); tv.append(m.group()); tp.append(m.start());
te.append(m.end())
    return tt, tv, tp, te

_STRIP = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:[^"\\]|\\.)*"', re.S)
_MODSCAN = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:[^"\\]|\\.)*"|\b(macromodule|module|endmodule)\b',
re.S)
_ELSE = re.compile(r'\s*else\b')
_KW = re.compile(r'\b(begin|end|case|casex|casez|endcase|generate|endgenerate|function|endfunction|task|endtask|specify|endspecify|fork|join)\b')

class JobCancelled(BaseException):          # BaseException: broad
'except Exception' handlers must not swallow a cancel
    pass

def _balanced(seg):
    s = _STRIP.sub(' ', seg)
    if s.count('(') != s.count(')') or s.count('{') != s.count('}') or
s.count('[') != s.count(']'): return False
    c = {}
    for w in _KW.findall(s): c[w] = c.get(w, 0) + 1
    g = c.get
    return (g('begin', 0) == g('end', 0) and g('case', 0) + g('casex',
0) + g('casez', 0) == g('endcase', 0) and g('generate', 0) ==
g('endgenerate', 0)
            and g('function', 0) == g('endfunction', 0) and g('task',
0) == g('endtask', 0) and g('specify', 0) == g('endspecify', 0) and
g('fork', 0) == g('join', 0))

class LineMap:
    """maps lines of the written design back to (module, instance)"""
    def __init__(self): self.starts = []; self.items = []
    def add(self, start, n, mod, kind, data):
self.starts.append(start); self.items.append((n, mod, kind, data))
    def lookup(self, line):
        i = bisect.bisect_right(self.starts, line) - 1
        if i < 0: return None
        n, mod, kind, data = self.items[i]; rel = line - self.starts[i]
        if callable(data): data = data(); self.items[i] = (n, mod, kind, data)
        if rel < 0 or rel >= n: return None
        if kind == 'list': return (mod, data[rel])
        starts, ends, names = data; j = bisect.bisect_right(starts, rel) - 1
        if j >= 0 and rel <= ends[j]: return (mod, names[j])
        return (mod, None)

class Net:
    __slots__ = ('name', 'msb', 'lsb', 'vec', 'type', 'dir',
'raw_range', 'declared')
    def __init__(self, name):
        self.name = name; self.msb = 0; self.lsb = 0; self.vec =
False; self.type = ''; self.dir = None; self.raw_range = '';
self.declared = False

class Port:
    __slots__ = ('name', 'dir', 'type', 'declared')
    def __init__(self, name, dir_='input', type_='', declared=True):
        self.name = name; self.dir = dir_; self.type = type_;
self.declared = declared

class Conn:
    __slots__ = ('pin', 'expr')
    def __init__(self, pin, expr): self.pin = pin; self.expr = expr

class Inst:
    __slots__ = ('name', 'type', 'params', 'attrs', 'conns',
'positional', 'prim', 'arr', 'line', 'l2d', 'lz')
    def __init__(self, name, type_):
        self.name = name; self.type = type_; self.params = '';
self.attrs = ''; self.conns = []
        self.positional = False; self.prim = False; self.arr = '';
self.line = 0; self.l2d = 0; self.lz = None
    @property
    def line2(self): return self.line + self.l2d
    @line2.setter
    def line2(self, v): self.l2d = v - self.line

class Assign:
    __slots__ = ('lhs', 'rhs')
    def __init__(self, lhs, rhs): self.lhs = lhs; self.rhs = rhs

SRC_Z_MIN = 2000000
class Module:
    @property
    def src(self):
        d = self.__dict__; z = d.get('srcz')
        return zlib.decompress(z).decode('utf-8') if z is not None
else d.get('srcp', '')
    @src.setter
    def src(self, t):
        d = self.__dict__
        if len(t) >= SRC_Z_MIN: d['srcz'] =
zlib.compress(t.encode('utf-8'), 1); d['srcp'] = ''
        else: d['srcz'] = None; d['srcp'] = t
    def set_src_from(self, text, a, b):
        d = self.__dict__
        if b - a < SRC_Z_MIN: self.src = text[a:b]; return
        co = zlib.compressobj(1); out = []; step = 4 << 20
        for i in range(a, b, step):
out.append(co.compress(text[i:min(b, i + step)].encode('utf-8')))
        out.append(co.flush()); d['srcz'] = b''.join(out); d['srcp'] = ''
    def __init__(self, name):
        self.name = name; self.line = 0; self.lead = '\n\n'; self.src
= ''; self.dirty = True; self.ansi = True
        self.params = ''; self.ports = []; self.nets = {};
self.net_order = []; self.param_lines = []
        self.assigns = []; self.insts = []; self.raw = []; self.dups =
[]; self.layout = {}
        self._bc = None; self._view = None; self._sym = None
    def __deepcopy__(self, memo):
        n = Module.__new__(Module)
        for k, v in self.__dict__.items():
            n.__dict__[k] = None if k.startswith('_') else
copy.deepcopy(v, memo)
        return n
    def __getstate__(self):
        return {k: v for k, v in self.__dict__.items() if not k.startswith('_')}
    def __setstate__(self, st):
        self.__dict__.update(st); self._bc = None; self._view = None;
self._sym = None; self._cix = None; self._ix = None
    def inst(self, name):
        L = self.insts
        if isinstance(L, LazyInsts): return L.find(name)
        if len(L) > 400:
            ix = self.__dict__.get('_ix')
            if ix is None or ix[0] != len(L) or ix[2] != _IX_EPOCH[0]:
                ix = self._ix = (len(L), {i.name: i for i in L}, _IX_EPOCH[0])
            i = ix[1].get(name)
            if i is not None and i.name == name: return i
            return None
        for i in L:
            if i.name == name: return i
        return None

_IX_EPOCH = [0]          # bumped when an undo renames an instance
behind the name indexes' back
def ix_set(m, inst, present):
    """keep a module's instance-name index current after a single add
/ remove (a rebuild costs O(instances))"""
    L = m.insts
    if isinstance(L, LazyInsts):
        if present:
            if inst.name: L.named[inst.name] = inst
        elif L.named.get(inst.name) is inst: del L.named[inst.name]
        return
    ix = m.__dict__.get('_ix')
    if ix is None: return
    d = ix[1]
    if present: d[inst.name] = inst
    elif d.get(inst.name) is inst: del d[inst.name]
    m._ix = (len(m.insts), d, ix[2])

WIRE_ROWS_MAX = 2000        # the inspector lists at most this many
wires of a module (millions of Tk rows freeze the GUI)

LAZY_BYTES = int(os.environ.get('NS_LAZY_BYTES', 12 << 20))     #
modules whose text is at least this big keep their instances as text
offsets

class LazyInsts:
    """The instances of a huge module, kept as offsets into the module
text (about 25 bytes each instead of ~500).
    Behaves like the list it replaces; Inst objects are built on
demand.  Objects handed out by lookup (inst()) or
    registered for an edit (jinst) are 'pinned': they stay canonical,
so edits made to them persist."""
    def __init__(self):
        self.m = None; self.text = ''; self.nlp = None
        self.st = array('I'); self.en = array('I'); self.ns =
array('I'); self.nlen = array('H'); self.ty = array('I')
        self.tnames = []; self.tid = {}
        self.seq = array('i'); self.extras = []; self.dead = set();
self.cache = {}; self.named = {}; self.otype = {}; self.pool = {}
        self._hidx = None

    # ---- construction (parser) ----
    def finish(self, m, text, off, nl_abs, a, b):
        self.m = m; self.text = text; nlp = array('I'); a0 =
bisect.bisect_right(nl_abs, a); a1 = bisect.bisect_right(nl_abs, b)
        for k in range(a0, a1, 1 << 20): nlp.extend(array('I', [x - a
for x in nl_abs[k:min(a1, k + (1 << 20))]]))
        self.nlp = nlp

    # ---- materialising ----
    def name_at(self, slot):
        if slot < 0: return self.extras[-1 - slot].name
        o = self.cache.get(slot)
        if o is not None: return o.name
        n = self.ns[slot]; return self.text[n:n + self.nlen[slot]]
    def type_at(self, slot):
        if slot < 0: return self.extras[-1 - slot].type
        o = self.cache.get(slot)
        return o.type if o is not None else self.tnames[self.ty[slot]]
    def pairs(self, slot):
        """[(pin, expr)] of a base statement, read straight from the text"""
        return self._pairs(_FINST.match(self.text, self.st[slot],
self.en[slot]))
    @staticmethod
    def _pairs(mt):
        return [(pn, _WSRX.sub(' ', ex).strip() if ex and (ex[0] <= '
' or ex[-1] <= ' ' or '\n' in ex or '  ' in ex) else ex) for pn, ex in
_FPIN.findall(mt.group(3))]
    def _mat(self, slot):
        text = self.text; mt = _FINST.match(text, self.st[slot],
self.en[slot]); I = self.pool.setdefault
        inst = Inst(mt.group(2), I(mt.group(1), mt.group(1)))
        inst.conns = [Conn(I(pn, pn), I(ex, ex)) for pn, ex in self._pairs(mt)]
        nlp = self.nlp; bis = bisect.bisect_right
        inst.line = bis(nlp, self.st[slot]); inst.l2d = bis(nlp,
mt.start(4)) - inst.line; inst.lz = (self, slot)
        return inst
    def get(self, slot):
        if slot < 0: return self.extras[-1 - slot]
        o = self.cache.get(slot)
        return o if o is not None else self._mat(slot)
    def pin(self, slot, inst):
        if slot >= 0 and slot not in self.cache:
            self.otype.setdefault(slot, inst.type); self.cache[slot] = inst
    def slot_of(self, inst):
        lz = inst.lz
        if lz is not None and lz[0] is self: return lz[1]
        for k, o in enumerate(self.extras):
            if o is inst: return -1 - k
        return None

    # ---- list protocol ----
    def __len__(self): return len(self.seq)
    def __bool__(self): return len(self.seq) > 0
    def __iter__(self):
        get = self.get
        for slot in self.seq: yield get(slot)
    def __reversed__(self):
        get = self.get
        for k in range(len(self.seq) - 1, -1, -1): yield get(self.seq[k])
    def __getitem__(self, i):
        if isinstance(i, slice): return [self.get(x) for x in self.seq[i]]
        return self.get(self.seq[i])
    def append(self, inst):
        self.extras.append(inst); self.seq.append(-len(self.extras))
        if inst.name: self.named.setdefault(inst.name, inst)
    def _extra_slot(self, inst):
        lz = inst.lz
        if lz is not None and lz[0] is self and lz[1] in self.dead:
self.dead.discard(lz[1]); return lz[1]
        for k in range(len(self.extras) - 1, -1, -1):
            if self.extras[k] is inst:
                if -1 - k in self.dead: self.dead.discard(-1 - k); return -1 - k
                break
        self.extras.append(inst); return -len(self.extras)
    def insert(self, idx, inst):
        slot = self._extra_slot(inst); self.seq.insert(idx, slot)
        if inst.name: self.named[inst.name] = inst
    def __delitem__(self, idx):
        slot = self.seq[idx]; del self.seq[idx]; self.dead.add(slot)
        o = self.get(slot) if slot < 0 else self.cache.get(slot)
        if o is not None and self.named.get(o.name) is o: del self.named[o.name]
    def pop(self, idx=-1):
        o = self.get(self.seq[idx]); del self[idx]; return o
    def __setitem__(self, idx, inst):
        old = self.seq[idx]; self.dead.add(old);
self.extras.append(inst); self.seq[idx] = -len(self.extras)
        if inst.name: self.named[inst.name] = inst

    # ---- lookup by name ----
    def _hash_index(self):
        h = self._hidx
        if h is None:
            n = len(self.st); shift = max(1, n.bit_length()); mask =
(1 << (62 - shift)) - 1; text = self.text; ns = self.ns; nl =
self.nlen
            L = [((hash(text[ns[i]:ns[i] + nl[i]]) & mask) << shift) |
i for i in range(n)]; L.sort()
            h = self._hidx = (array('q', L), shift, mask)
        return h
    def find(self, name):
        o = self.named.get(name)
        if o is not None and o.name == name: return o
        P, shift, mask = self._hash_index(); hh = hash(name) & mask;
lo = bisect.bisect_left(P, hh << shift); n = len(P); em = (1 << shift)
- 1
        while lo < n and (P[lo] >> shift) == hh:
            slot = P[lo] & em; lo += 1
            if slot in self.dead: continue
            c = self.cache.get(slot)
            if c is not None:
                if c.name == name: return c
                continue
            nn = self.name_at(slot)
            if nn == name:
                o = self._mat(slot); self.pin(slot, o); return o
        return None
    def index_of(self, inst):
        slot = self.slot_of(inst)
        if slot is None: return -1
        try: return self.seq.index(slot)
        except ValueError: return -1
    def match(self, rx):
        """instances whose name matches the compiled regex, without
building the others"""
        out = []; name_at = self.name_at; get = self.get
        for slot in self.seq:
            if rx.match(name_at(slot)): out.append(get(slot))
        return out

    # ---- types ----
    def type_counts(self):
        from collections import Counter
        base = getattr(self, '_tc', None)
        if base is None:
            base = Counter(self.ty); self._tc = base
 # base never changes: counted once, at C speed
        c = Counter({self.tnames[k]: v for k, v in base.items()}); tn
= self.tnames; ty = self.ty
        for slot in self.dead:
            if slot >= 0: c[tn[ty[slot]]] -= 1
        for slot, o in self.cache.items():
            if slot not in self.dead: c[tn[ty[slot]]] -= 1; c[o.type] += 1
        dead = self.dead
        for k, o in enumerate(self.extras):
            if -1 - k not in dead: c[o.type] += 1
        return +c
    def type_set(self): return set(self.type_counts())
    def of_types(self, types):
        out = []; tn = self.tnames; ty = self.ty; get = self.get;
cache = self.cache
        want = {i for i, t in enumerate(tn) if t in types}
        if not want:                         # no untouched statement
can match: only pinned / added instances remain
            for slot in self.seq:
                if slot < 0 or slot in cache:
                    o = get(slot)
                    if o.type in types: out.append(o)
            return out
        for slot in self.seq:
            if slot >= 0 and slot not in cache:
                if ty[slot] in want: out.append(get(slot))
            else:
                o = get(slot)
                if o.type in types: out.append(o)
        return out

    # ---- writing ----
    def iter_out(self):
        """(statement text | None, Inst | None) for every instance in
order: untouched statements are emitted verbatim"""
        text = self.text; st = self.st; en = self.en; cache = self.cache
        for slot in self.seq:
            if slot >= 0 and slot not in cache: yield
text[st[slot]:en[slot]], None
            else: yield None, self.get(slot)
    def exact(self):
        """(starts, ends, names) for the line map of an unmodified module"""
        nlp = self.nlp; bis = bisect.bisect_right; S = array('i'); E =
array('i'); seq = self.seq
        for slot in seq:
            if slot >= 0:
                a = bis(nlp, self.st[slot]); S.append(a); E.append(a +
(bis(nlp, self.en[slot] - 1) - a))
            else:
                o = self.extras[-1 - slot]; S.append(o.line); E.append(o.line2)
        lz = self
        class Names:
            def __getitem__(s, j): return lz.name_at(seq[j])
            def __len__(s): return len(seq)
        return S, E, Names()


class Design:
    def __init__(self):
        self.modules = {}; self.order = []; self.tail = ''; self.errors = []
        self.eco_net_idx = 0; self.eco_port_idx = 0; self.eco_log = []

def touch(m):
    m.dirty = True; m._bc = None; m._view = None; m._sym = None;
m._cix = None; m._ix = None

_CIDX_LIVE = weakref.WeakSet()
def note_dirty(*insts):
    """tell every compact connectivity index that these instances
changed (their stored entries are stale)"""
    for c in _CIDX_LIVE: c.mark(insts)
def soft_touch(m, *insts):
    """an edit that changes only the given instances / adds unused
nets: keep a compact connectivity index and patch it"""
    m.dirty = True; m._bc = None; m._view = None; m._sym = None
    if insts: note_dirty(*insts)
    c = getattr(m, '_cix', None)
    if c is not None and not isinstance(c[0], CIdx): m._cix = None

# ---- change journal: cheap undo/redo for modules too big to snapshot
(JOURNAL_MIN instances) ----
JOURNAL_MIN = 60000
JRN = None          # the journal that is recording the current edit, or None
BIGMARK = object()  # snapshot placeholder for journalled modules

class _InstItem:
    __slots__ = ('inst', 'st')
    def __init__(self, inst):
        self.inst = inst; self.st = self.cap()
    def cap(self):
        i = self.inst; return (i.name, i.type, i.params, i.attrs,
i.positional, i.prim, [(c, c.pin, c.expr) for c in i.conns])
    def swap(self):
        cur = self.cap(); i = self.inst; note_dirty(i)
        if i.name != self.st[0]: _IX_EPOCH[0] += 1
        i.name, i.type, i.params, i.attrs, i.positional, i.prim, cs = self.st
        for c, pn, ex in cs: c.pin = pn; c.expr = ex
        i.conns = [c for c, _, _ in cs]; self.st = cur
class _PresItem:
    __slots__ = ('m', 'inst', 'idx', 'present')
    def __init__(self, m, inst, idx, present): self.m = m; self.inst =
inst; self.idx = idx; self.present = present
    def swap(self):
        L = self.m.insts
        if self.present:
            if self.idx < len(L) and L[self.idx] is self.inst: del L[self.idx]
            elif isinstance(L, LazyInsts):
                k = L.index_of(self.inst)
                if k >= 0: del L[k]; self.idx = k
            else:
                for k, x in enumerate(L):
                    if x is self.inst: del L[k]; self.idx = k; break
        else: L.insert(min(self.idx, len(L)), self.inst)
        self.present = not self.present; ix_set(self.m, self.inst,
self.present); soft_touch(self.m, self.inst)
class _NetItem:
    __slots__ = ('m', 'name', 'st')
    def __init__(self, m, name): self.m = m; self.name = name; self.st
= self.cap()
    def cap(self):
        n = self.m.nets.get(self.name)
        return None if n is None else (n.msb, n.lsb, n.vec, n.type,
n.dir, n.raw_range, n.declared)
    def swap(self):
        cur = self.cap(); m = self.m
        if self.st is None: m.nets.pop(self.name, None)
        else:
            n = m.nets.get(self.name)
            if n is None: n = Net(self.name); m.nets[self.name] = n
            n.msb, n.lsb, n.vec, n.type, n.dir, n.raw_range,
n.declared = self.st
        self.st = cur; touch(m)
class _ListItem:
    """lists that edits either append to in place or replace
wholesale; net_order may hold millions of names, so it is never
copied"""
    __slots__ = ('m', 'attr', 'saved')
    def __init__(self, m, attr):
        self.m = m; self.attr = attr; L = getattr(m, attr)
        self.saved = (L, len(L), None) if attr == 'net_order' else
(list(L), 0, None)
    def swap(self):
        m = self.m; cur_obj = getattr(m, self.attr); sobj, sn, stail =
self.saved
        if self.attr == 'net_order':
            if cur_obj is sobj: tail = list(cur_obj[sn:]); del
cur_obj[sn:]; cur = (sobj, sn, tail)
            else: cur = (cur_obj, len(cur_obj), None)
            if stail: sobj.extend(stail)
            setattr(m, self.attr, sobj)
        else:
            cur = (list(cur_obj), 0, None); setattr(m, self.attr, list(sobj))
        self.saved = cur; touch(m)
class _ObjItem:
    __slots__ = ('obj', 'st')
    def __init__(self, obj): self.obj = obj; self.st = dict((k,
getattr(obj, k)) for k in obj.__slots__)
    def swap(self):
        cur = dict((k, getattr(self.obj, k)) for k in self.obj.__slots__)
        for k, v in self.st.items(): setattr(self.obj, k, v)
        self.st = cur

class Journal:
    def __init__(self): self.items = []; self.seen = set(); self.rev = True
    def add(self, key, make):
        if key in self.seen: return
        self.seen.add(key); self.items.append(make())
    def swap(self):
        for it in (reversed(self.items) if self.rev else self.items): it.swap()
        self.rev = not self.rev
def jinst(inst):
    lz = inst.lz
    if lz is not None: lz[0].pin(lz[1], inst)         # an instance
about to be edited stays canonical
    if JRN is not None: JRN.add(('i', id(inst)), lambda: _InstItem(inst))
def jnet(m, name):
    if JRN is not None: JRN.add(('n', id(m), name), lambda: _NetItem(m, name))
def jlist(m, attr):
    if JRN is not None: JRN.add(('l', id(m), attr), lambda: _ListItem(m, attr))
def jobj(o):
    if JRN is not None: JRN.add(('o', id(o)), lambda: _ObjItem(o))
def jpres(m, inst, idx, present):
    if JRN is not None: JRN.items.append(_PresItem(m, inst, idx, present))

def net_width(n):
    if n is None: return 1
    return abs(n.msb - n.lsb) + 1 if n.vec and n.msb == n.msb else 1

def declare_net(m, name, rng, type_, dir_):
    n = m.nets.get(name)
    if n:
        if not dir_ and not n.dir and n.declared: m.dups.append(name)
        if rng: n.msb, n.lsb, n.vec, n.raw_range = rng[0], rng[1], True, rng[2]
        if type_ and (not n.type or n.type == 'wire'): n.type = type_
        if dir_: n.dir = dir_
        if not dir_: n.declared = True
        return n
    n = Net(name); n.type = type_ or ''; n.dir = dir_ or None;
n.declared = not dir_
    if rng: n.msb, n.lsb, n.vec, n.raw_range = rng[0], rng[1], True, rng[2]
    m.nets[name] = n; m.net_order.append(name)
    return n

_FID = r'[A-Za-z_][A-Za-z0-9_$]*'
_FWS = r'[ \t\r\n]*'
# plain "TYPE name ( .pin(expr), ... );" with named connections only,
no params/arrays/escapes/comments/macros inside
_FCONN = r'\.' + _FWS + _FID + _FWS + r'\([^()\\/"`;]*\)'
_FINST = re.compile(_FWS + '(' + _FID + r')[ \t\r\n]+(' + _FID + ')' +
_FWS + r'\((' + _FWS + _FCONN + '(?:' + _FWS + ',' + _FWS + _FCONN +
')*)' + _FWS + r'(\))' + _FWS + ';')
_FPIN = re.compile(r'\.\s*(' + _FID + r')\s*\(([^()]*)\)')
_FWIRE = re.compile(_FWS + r'wire(?:[ \t]+\[(\d+):(\d+)\])?[
\t\r\n]+(' + _FID + '(?:' + _FWS + ',' + _FWS + _FID + ')*)' + _FWS +
';')
_FSPLIT = re.compile(r'\s*,\s*')
_WSRX = re.compile(r'\s+')
_FSKIP = frozenset(('module', 'macromodule', 'function', 'task',
'generate', 'begin', 'end', 'assign', 'always', 'initial', 'input',
'output', 'inout', 'parameter', 'localparam', 'defparam', 'specify',
'if', 'else', 'for', 'case', 'wire', 'reg', 'tri', 'supply0',
'supply1', 'integer', 'real', 'genvar'))

FAST_PARSE = os.environ.get('NS_FAST_PARSE', '0') == '1'      # off:
the plain token parser reads every netlist (set NS_FAST_PARSE=1 to
re-enable the text fast path, lazy instances and parallel scan)

PAR_SCAN_MIN = int(os.environ.get('NS_PAR_SCAN_MIN', 24 << 20))      #
text this big is scanned by several processes

def _scan_fast(text, a, b, off):
    """worker: consume plain instance / wire statements in text[a:b];
-> (arrays as bytes, local type names, wires, end offset)"""
    fi = _FINST.match; fw = _FWIRE.match; st = array('I'); en =
array('I'); ns_ = array('I'); nl_ = array('H'); ty = array('I')
    tn = []; tid = {}; wires = []; pos = a
    while pos < b:
        mt = fi(text, pos, b)
        if mt:
            t0 = mt.group(1); t = tid.get(t0)
            if t is None:
                if t0 in _FSKIP or t0.lower() in PRIMS: t = -1
                else: t = len(tn); tn.append(t0)
                tid[t0] = t
            if t < 0: break
            s1 = mt.start(2); e0 = mt.end(); st.append(mt.start(1) -
off); en.append(e0 - off); ns_.append(s1 - off); nl_.append(mt.end(2)
- s1); ty.append(t); pos = e0
        else:
            mt = fw(text, pos, b)
            if not mt: break
            wires.append((mt.group(1), mt.group(2), mt.group(3))); pos
= mt.end()
    return st.tobytes(), en.tobytes(), ns_.tobytes(), nl_.tobytes(),
ty.tobytes(), tn, wires, pos


class VParser:
    CHUNK = 1500000
    def __init__(self, text, progress=None, cancel=None):
        self.chunk = 16384; self.fast_hits = 0; self.lz = None; self.lz_off = 0
        self.text = text; self.progress = progress; self.cancel =
cancel; self.pool = {}
        self.tt = []; self.tv = []; self.tp = []; self.te = []; self.n
= 0; self.p = 0; self.win_end = 0
        self.design = Design()
        self.nl = array('I'); find = text.find; i = -1; app = self.nl.append
        while True:
            i = find('\n', i + 1)
            if i < 0: break
            app(i + 1)
        self.nl.insert(0, 0)
    def I(self, s):
        return self.pool.setdefault(s, s)
    def line_at(self, pos):
        return bisect.bisect_right(self.nl, pos)
    def window(self, a, b):
        self.tt, self.tv, self.tp, self.te = lex(self.text, a, b);
self.n = len(self.tt); self.p = 0; self.win_end = b
    def err(self, msg, tok=None):
        pos = 0
        if self.n:
            t = min(self.p if tok is None else tok, self.n - 1); pos =
self.tp[t]
        self.design.errors.append((self.line_at(pos), msg))
    def next_window(self, a, mend):
        """end of a window starting at a that stops on a statement boundary"""
        ck = self.chunk
        if mend - a <= ck: return mend
        b = a + ck; text = self.text
        while b < mend:
            k = text.rfind(';', a, b); tries = 0
            while k >= a and tries < 300:
                if not _ELSE.match(text, k + 1) and _balanced(text[a:k
+ 1]): return k + 1
                k = text.rfind(';', a, k); tries += 1
            b = min(mend, b + ck)
        return mend
    def slice(self, a, b): return self.text[self.tp[a]:self.te[b]]
    @staticmethod
    def norm(s): return re.sub(r'\s+', ' ', s).strip()
    def is_name(self, i): return i < self.n and self.tt[i] in (T_ID, T_ESC)
    def name(self):
        if not self.is_name(self.p):
            self.err('Expected an identifier but found "%s"' %
(self.tv[self.p] if self.p < self.n else 'end of file'))
            return ''
        v = self.tv[self.p]; self.p += 1; return v
    def tok(self, i=None):
        i = self.p if i is None else i
        return self.tv[i] if i < self.n else ''
    def skip_bal(self):
        o = self.tok(); c = {'(': ')', '[': ']', '{': '}'}[o]; d = 0
        while self.p < self.n:
            if self.tt[self.p] == T_P:
                if self.tv[self.p] == o: d += 1
                elif self.tv[self.p] == c:
                    d -= 1
                    if d == 0: self.p += 1; return
            self.p += 1
    def to_semi(self):
        d = 0
        while self.p < self.n:
            v = self.tv[self.p]
            if self.tt[self.p] == T_P:
                if v in '([{': d += 1
                elif v in ')]}': d -= 1
                elif v == ';' and d <= 0: self.p += 1; return
            self.p += 1
    def range_(self):
        s = self.p; self.skip_bal()
        inner = self.norm(self.slice(s, self.p - 1)[1:-1]); k =
inner.find(':'); a = b = float('nan')
        if k > 0 and re.match(r'^\s*\d+\s*$', inner[:k]) and
re.match(r'^\s*\d+\s*$', inner[k + 1:]):
            a = int(inner[:k]); b = int(inner[k + 1:])
        return (a, b, '[' + inner + ']')
    def skip_stmt(self):
        v = self.tok()
        if self.tt[self.p] == T_ID and v == 'begin':
            d = 1; self.p += 1
            while d and self.p < self.n:
                if self.tv[self.p] == 'begin': d += 1
                elif self.tv[self.p] == 'end': d -= 1
                self.p += 1
            return
        if v in ('case', 'casez', 'casex'):
            while self.p < self.n and self.tv[self.p] != 'endcase': self.p += 1
            self.p += 1; return
        if v == 'if':
            self.p += 1
            if self.tok() == '(': self.skip_bal()
            self.skip_stmt()
            if self.tok() == 'else': self.p += 1; self.skip_stmt()
            return
        if v in ('for', 'while', 'repeat', 'forever'):
            self.p += 1
            if self.tok() == '(': self.skip_bal()
            self.skip_stmt(); return
        self.to_semi()
    def raw_until(self, end):
        s = self.p
        while self.p < self.n and self.tv[self.p] != end: self.p += 1
        e = self.p; self.p += 1
        return self.slice(s, min(e, self.n - 1))
    def parse_ports(self, m):
        self.p += 1
        if self.tok() == ')': self.p += 1; return
        ansi = self.tok() in ('input', 'output', 'inout'); m.ansi = ansi
        if not ansi:
            while self.p < self.n and self.tok() != ')':
                if self.is_name(self.p):
m.ports.append(Port(self.tv[self.p], 'input', '', False))
                self.p += 1
            self.p += 1; return
        cur = ['input', '', None]
        while self.p < self.n and self.tok() != ')':
            if self.tt[self.p] == T_ATTR: self.p += 1; continue
            if self.tok() in ('input', 'output', 'inout'): cur =
[self.tok(), '', None]; self.p += 1
            ty = []
            while self.p < self.n and self.tt[self.p] == T_ID and
(self.tv[self.p] in NET_TYPES or self.tv[self.p] in ('signed',
'unsigned', 'var')):
                ty.append(self.tv[self.p]); self.p += 1
            if ty: cur[1] = ' '.join(ty)
            if self.tok() == '[': cur[2] = self.range_()
            nm = self.name()
            if not nm: break
            while self.tok() == '[': self.skip_bal()
            t = '' if cur[1] == 'wire' else cur[1]
            m.ports.append(Port(nm, cur[0], t)); declare_net(m, nm,
cur[2], t, cur[0])
            if self.tok() == ',': self.p += 1
            elif self.tok() != ')': self.err('Unexpected "%s" in port
list' % self.tok()); self.p += 1
        self.p += 1
    def decl_port(self, m):
        d = self.tv[self.p]; self.p += 1; ty = []
        while self.p < self.n and self.tt[self.p] == T_ID and
(self.tv[self.p] in NET_TYPES or self.tv[self.p] in ('signed',
'unsigned')):
            ty.append(self.tv[self.p]); self.p += 1
        rng = self.range_() if self.tok() == '[' else None
        while self.p < self.n and self.tok() != ';':
            if self.is_name(self.p):
                nm = self.tv[self.p]; self.p += 1
                pt = None
                for q in m.ports:
                    if q.name == nm: pt = q
                t = ' '.join(x for x in ty if x != 'wire')
                if pt is None: pt = Port(nm, d, t); m.ports.append(pt)
                pt.dir = d; pt.type = t; pt.declared = True
                declare_net(m, nm, rng, t, d)
                while self.tok() == '[': self.skip_bal()
            else: self.p += 1
        self.p += 1
    def decl_net(self, m):
        ty = self.tv[self.p]; self.p += 1
        if ty == 'wire': ty = ''
        while self.p < self.n and self.tt[self.p] == T_ID and
self.tv[self.p] in ('signed', 'unsigned', 'wire', 'reg'): self.p += 1
        rng = self.range_() if self.tok() == '[' else None
        while self.p < self.n and self.tok() != ';':
            if self.is_name(self.p):
                nm = self.tv[self.p]; self.p += 1
                while self.tok() == '[': self.skip_bal()
                declare_net(m, nm, rng, ty, None)
                if self.tok() == '=':
                    self.p += 1; s = self.p; d = 0
                    while self.p < self.n:
                        if self.tt[self.p] == T_P:
                            v = self.tv[self.p]
                            if v in '({[': d += 1
                            elif v in ')}]': d -= 1
                            elif v in ',;' and d <= 0: break
                        self.p += 1
                    m.assigns.append(Assign(nm,
self.norm(self.slice(s, self.p - 1))))
            else: self.p += 1
            if self.tok() == ',': self.p += 1
        self.p += 1
    def parse_assign(self, m):
        self.p += 1
        if self.tok() == '#':
            self.p += 1
            if self.tok() == '(': self.skip_bal()
            else: self.p += 1
        if self.tok() == '(' and self.tok(self.p + 1) in ('strong0',
'weak0', 'supply0', 'strong1'): self.skip_bal()
        while self.p < self.n and self.tok() != ';':
            s = self.p; d = 0
            while self.p < self.n:
                if self.tt[self.p] == T_P:
                    v = self.tv[self.p]
                    if v in '({[': d += 1
                    elif v in ')}]': d -= 1
                    elif v == '=' and d <= 0: break
                self.p += 1
            lhs = self.norm(self.slice(s, self.p - 1)); self.p += 1
            s = self.p; d = 0
            while self.p < self.n:
                if self.tt[self.p] == T_P:
                    v = self.tv[self.p]
                    if v in '({[': d += 1
                    elif v in ')}]': d -= 1
                    elif v in ',;' and d <= 0: break
                self.p += 1
            m.assigns.append(Assign(lhs, self.norm(self.slice(s, self.p - 1))))
            if self.tok() == ',': self.p += 1
        self.p += 1
    def parse_conns(self):
        cs = []
        if self.tok() == ')': return cs
        while True:
            if self.tok() == '.':
                self.p += 1
                if self.tok() == '*': cs.append(Conn('*', '')); self.p += 1
                else:
                    pin = self.name(); ex = ''
                    if self.tok() == '(':
                        s = self.p + 1; self.skip_bal()
                        ex = self.norm(self.slice(s, self.p - 2)) if
self.p - 1 > s else ''
                    cs.append(Conn(self.I(pin), self.I(ex)))
            else:
                s = self.p; d = 0
                while self.p < self.n:
                    if self.tt[self.p] == T_P:
                        v = self.tv[self.p]
                        if v in '({[': d += 1
                        elif v in ')}]':
                            if d == 0: break
                            d -= 1
                        elif v == ',' and d == 0: break
                    self.p += 1
                cs.append(Conn(None, self.I(self.norm(self.slice(s,
self.p - 1))) if self.p > s else ''))
            if self.tok() == ',': self.p += 1
            else: break
        return cs
    def par_fast(self, m, lz, pos, mend):
        """scan the plain statements from pos on several processes; ->
new offset (statements are appended in order)"""
        text = self.text; K = n_workers() * 2; span = mend - pos
        fi = _FINST.match; fw = _FWIRE.match
        cuts = [pos]
        for c in range(1, K):
            p = text.find('\n', pos + span * c // K) + 1
            for _ in range(400):
                if p <= 0 or p >= mend: p = -1; break
                if fi(text, p, mend) or fw(text, p, mend): break
                p = text.find('\n', p) + 1
            else: p = -1
            if p > cuts[-1]: cuts.append(p)
        cuts.append(mend)
        n = len(cuts) - 1
        if n < 2: return pos
        off = self.lz_off
        res = fork_map(lambda i: _scan_fast(text, cuts[i], cuts[i +
1], off), n, self.cancel)
        end = pos
        for i, r in enumerate(res):
            st, en, ns_, nl_, ty, tn, wires, e = r
            remap = []
            for t0 in tn:
                t = lz.tid.get(t0)
                if t is None: t = lz.tid[t0] = len(lz.tnames);
lz.tnames.append(t0)
                remap.append(t)
            a = array('I'); a.frombytes(ty)
            first = len(lz.st)
            lz.st.frombytes(st); lz.en.frombytes(en);
lz.ns.frombytes(ns_); lz.nlen.frombytes(nl_)
            lz.ty.extend(array('I', [remap[x] for x in a]) if remap !=
list(range(len(remap))) else a)
            lz.seq.extend(range(first, len(lz.st)))
            nets = m.nets; order = m.net_order
            for g1, g2, g3 in wires:
                rng = (int(g1), int(g2), '[%s:%s]' % (g1, g2)) if g1
is not None else None
                for nm in _FSPLIT.split(g3):
                    nt = nets.get(nm)
                    if nt is None:
                        nt = Net(nm); nt.declared = True
                        if rng: nt.msb, nt.lsb, nt.vec, nt.raw_range =
rng[0], rng[1], True, rng[2]
                        nets[nm] = nt; order.append(nm)
                    else: declare_net(m, nm, rng, '', None)
            end = e
            if e < cuts[i + 1]: break                     # a
statement the fast path does not take: later chunks are discarded
        self.fast_hits += len(lz.st)
        return end
    def fast_run(self, m, pos, mend):
        """consume a run of plain instance / wire statements straight
from the text, without tokenizing; returns the new offset"""
        if not FAST_PARSE: return pos
        text = self.text; I = self.I; pins = _FPIN.findall; norm_ws =
_WSRX.sub; fi = _FINST.match; fw = _FWIRE.match; insts = m.insts; app
= insts.append
        lz = self.lz
        if lz is not None:
            if mend - pos >= PAR_SCAN_MIN and fork_ok(): pos =
self.par_fast(m, lz, pos, mend)
            st = lz.st; en = lz.en; nsA = lz.ns; nlA = lz.nlen; tyA =
lz.ty; seq = lz.seq; tid = lz.tid; off = self.lz_off; n0 = pos
        nl = self.nl; bis = bisect.bisect_right; base = m.line; n0 =
pos; cnt = 0; nets = m.nets; order = m.net_order
        while pos < mend:
            mt = fi(text, pos, mend)
            if mt and lz is not None:
                ty = mt.group(1); t = tid.get(ty)
                if t is None:
                    if ty in _FSKIP or ty.lower() in PRIMS: t = -1
                    else: t = len(lz.tnames); lz.tnames.append(ty)
                    tid[ty] = t
                if t < 0: break
                s1 = mt.start(2); e0 = mt.end(); seq.append(len(st));
st.append(mt.start(1) - off); en.append(e0 - off); nsA.append(s1 -
off); nlA.append(mt.end(2) - s1); tyA.append(t)
                pos = e0; cnt += 1
            elif mt:
                ty = mt.group(1)
                if ty in _FSKIP or ty.lower() in PRIMS: break
                cs = [Conn(I(pn), I(norm_ws(' ', ex).strip() if ex and
(ex[0] <= ' ' or ex[-1] <= ' ' or '\n' in ex or '  ' in ex) else ex))
for pn, ex in pins(mt.group(3))]
                inst = Inst(mt.group(2), I(ty)); inst.conns = cs
                l1 = bis(nl, mt.start(1)); inst.line = l1 - base;
inst.l2d = bis(nl, mt.start(4)) - l1
                app(inst); pos = mt.end(); cnt += 1
            else:
                mt = fw(text, pos, mend)
                if not mt: break
                rng = (int(mt.group(1)), int(mt.group(2)), '[%s:%s]' %
(mt.group(1), mt.group(2))) if mt.group(1) is not None else None
                for nm in _FSPLIT.split(mt.group(3)):
                    n = nets.get(nm)
                    if n is None:
                        n = Net(nm); n.declared = True
                        if rng: n.msb, n.lsb, n.vec, n.raw_range =
rng[0], rng[1], True, rng[2]
                        nets[nm] = n; order.append(nm)
                    else: declare_net(m, nm, rng, '', None)
                pos = mt.end(); cnt += 1
            if cnt & 8191 == 0:
                if self.cancel and self.cancel(): raise JobCancelled()
                if self.progress: self.progress(pos / max(1,
len(text)), 'parsing module %s (%d cells)' % (m.name, len(insts)))
        if pos > n0: self.fast_hits += cnt
        return pos
    def reseat(self, pos, mend):
        self.window(pos, self.next_window(pos, mend))
    def parse_inst(self, m, attrs):
        line = self.line_at(self.tp[self.p]); type_ =
self.I(self.tv[self.p]); self.p += 1
        prim = type_.lower() in PRIMS; params = ''
        if self.tok() == '#':
            s = self.p; self.p += 1
            if self.tok() == '(': self.skip_bal()
            else: self.p += 1
            params = self.norm(self.slice(s, self.p - 1))
        while True:
            nm = ''; arr = ''
            if self.tok() != '(': nm = self.name()
            if self.tok() == '[':
                s2 = self.p; self.skip_bal(); arr =
self.norm(self.slice(s2, self.p - 1))
            if self.tok() != '(':
                self.err('Expected "(" in instance ' + nm);
self.to_semi(); return
            self.p += 1
            cs = self.parse_conns()
            if self.tok() == ')': self.p += 1
            else: self.err('Missing ")" in instance ' + nm)
            positional = bool(cs) and cs[0].pin is None
            if prim or positional:
                for i, c in enumerate(cs):
                    if c.pin is None: c.pin = ('Y' if i == 0 else
'I%d' % i) if prim else None
            inst = Inst(self.I(nm), type_); inst.params = params;
inst.attrs = attrs or ''; inst.conns = cs
            inst.positional = positional; inst.prim = prim; inst.arr =
arr; inst.line = line - m.line; inst.line2 =
self.line_at(self.tp[max(0, self.p - 1)]) - m.line  # (setter after
line)
            m.insts.append(inst)
            if self.tok() == ',': self.p += 1; continue
            break
        if self.tok() == ';': self.p += 1
        else: self.err('Expected ";" after instance'); self.to_semi()
    def parse_module(self, mstart, mend):
        self.window(mstart, self.next_window(mstart, mend))
        line = self.line_at(mstart); self.p += 1
        nm = self.name(); m = Module(self.I(nm)); m.line = line; m.dirty = False
        self.lz = None
        if mend - mstart >= LAZY_BYTES and len(self.text) < (1 << 32)
and FAST_PARSE:
            self.lz = LazyInsts(); m.insts = self.lz; self.lz_off = mstart
        if self.tok() == '#':
            s = self.p; self.p += 1
            if self.tok() == '(': self.skip_bal()
            m.params = self.norm(self.slice(s, self.p - 1))
        if self.tok() == '(': self.parse_ports(m)
        if self.tok() == ';': self.p += 1
        else: self.err('Expected ";" after module header')
        attrs = ''; closed = False
        while True:
            while self.p < self.n and self.tok() != 'endmodule':
                v = self.tok(); t = self.tt[self.p]
                if t == T_ATTR: attrs = self.norm(v); self.p += 1; continue
                if t == T_DIR: self.p += 1; continue
                if v in ('input', 'output', 'inout'): self.decl_port(m)
                elif t == T_ID and v in NET_TYPES:
                    if v == 'wire' and not attrs:
                        r = self.fast_run(m, self.tp[self.p], mend)
                        if r > self.tp[self.p]: self.chunk = 16384;
self.reseat(r, mend); continue
                    self.decl_net(m)
                elif v in ('parameter', 'localparam', 'defparam'):
                    s0 = self.p; self.to_semi();
m.param_lines.append(self.norm(self.slice(s0, self.p - 1)))
                elif v == 'assign': self.parse_assign(m)
                elif v in ('always', 'initial', 'always_comb',
'always_ff', 'always_latch', 'final'):
                    s1 = self.p; self.p += 1
                    if self.tok() == '@':
                        self.p += 1
                        if self.tok() == '(': self.skip_bal()
                        else: self.p += 1
                    elif self.tok() == '#': self.p += 2
                    self.skip_stmt();
m.raw.append(self.text[self.tp[s1]:self.te[self.p - 1]])
                elif v == 'generate':
                    s3 = self.p; self.raw_until('endgenerate');
m.raw.append(self.text[self.tp[s3]:self.te[self.p - 1]])
                elif v in ('function', 'task'):
                    s4 = self.p; self.raw_until('endfunction' if v ==
'function' else 'endtask');
m.raw.append(self.text[self.tp[s4]:self.te[self.p - 1]])
                elif v == 'specify':
                    s5 = self.p; self.raw_until('endspecify');
m.raw.append(self.text[self.tp[s5]:self.te[self.p - 1]])
                elif v in ('for', 'if'):
                    s6 = self.p; self.skip_stmt();
m.raw.append(self.text[self.tp[s6]:self.te[self.p - 1]])
                elif t in (T_ID, T_ESC):
                    if t == T_ID and not attrs:
                        r = self.fast_run(m, self.tp[self.p], mend)
                        if r > self.tp[self.p]: self.chunk = 16384;
self.reseat(r, mend); continue
                    self.parse_inst(m, attrs); attrs = ''
                else: self.err('Unexpected "%s"' % v); self.p += 1
            if self.p < self.n: closed = True; break
            if self.win_end >= mend: break
            if self.cancel and self.cancel(): raise JobCancelled()
            if self.progress: self.progress(self.win_end / max(1,
len(self.text)), 'parsing module %s' % m.name)
            self.chunk = min(self.CHUNK, self.chunk * 2)       # the
fast path is not paying off here: tokenize in bigger windows
            self.window(self.win_end, self.next_window(self.win_end, mend))
        if not closed: self.err('Missing endmodule for ' + nm, 0)
        if self.lz is not None:
            raw = self.text[mstart:mend]; m.__dict__['srcp'] = raw;
m.__dict__['srcz'] = None
            self.lz.finish(m, raw, mstart, self.nl, mstart, mend);
self.lz = None
        else: m.set_src_from(self.text, mstart, mend)
        if not m.ansi:
            for q in m.ports:
                if not q.declared: self.err('Port "%s" has no
direction declaration' % q.name, 0)
        self.tt = self.tv = self.tp = self.te = []; self.n = 0
        return m
    def parse(self):
        import gc
        was = gc.isenabled(); gc.disable()
        try: return self._parse()
        finally:
            if was: gc.enable()
    def _parse(self):
        text = self.text; D = self.design; prev_end = 0; mods = []; cur = None
        for mm in _MODSCAN.finditer(text):
            g = mm.group(1)
            if g is None: continue
            if g in ('module', 'macromodule'):
                if cur is None: cur = mm.start()
            elif cur is not None: mods.append((cur, mm.end())); cur = None
        if cur is not None: mods.append((cur, len(text)))
        for a, b in mods:
            if self.cancel and self.cancel(): raise JobCancelled()
            m = self.parse_module(a, b); m.lead = text[prev_end:a]
            if m.name in D.modules: D.errors.append((m.line, 'Module
"%s" is defined more than once' % m.name))
            D.modules[m.name] = m; D.order.append(m.name); prev_end = b
            if self.progress: self.progress(b / max(1, len(text)),
'parsed module %s' % m.name)
        D.tail = text[prev_end:]
        return D

def parse_verilog(text, progress=None, cancel=None): return
VParser(text, progress, cancel).parse()

class LibCheckError(Exception):
    """the netlist uses leaf cells that the loaded libraries do not define"""
    def __init__(self, report): Exception.__init__(self, report);
self.report = report

def collect_leaf_cells(D, lib):
    """pre-processing check: every leaf (non-hierarchical,
non-primitive) cell type the netlist instantiates.
    -> (used {type: count}, missing {type: count}, no_footprint [type])"""
    from collections import Counter
    used = Counter()
    for mn in D.order:
        m = D.modules[mn]
        for t, n in type_counts(m).items():
            if t in D.modules or t.lower() in PRIMS: continue
            used[t] += n
    cells = lib.cells if lib else {}
    missing = {t: n for t, n in used.items() if t not in cells}
    nofp = sorted(t for t in used if t in cells and not
cell_footprint(cells[t]))
    return dict(used), missing, nofp

def lib_check_report(D, lib, used, missing, nofp, limit=25):
    L = []
    if missing:
        L.append('%d of %d leaf cell type(s) used by the netlist are
not in the loaded Liberty libraries (%d instances):' % (len(missing),
len(used), sum(missing.values())))
        for t, n in sorted(missing.items(), key=lambda x:
-x[1])[:limit]: L.append('   %-32s %9d instance(s)' % (t, n))
        if len(missing) > limit: L.append('   ... and %d more cell
type(s)' % (len(missing) - limit))
        if not lib: L.append('No Liberty file is loaded: read the
library file(s) first.')
        else: L.append('Libraries read: %d cells from the loaded
Liberty file(s).' % len(lib.cells))
    if nofp: L.append('%d cell type(s) have no cell_footprint
attribute (footprint equivalence is derived from pins and function):
%s' % (len(nofp), ', '.join(nofp[:12]) + (' ...' if len(nofp) > 12
else '')))
    return '\n'.join(L)

def merge_lib(dst, src, fname, notes):
    """union of cells; the first definition of a cell wins, later
duplicates are reported"""
    dup = 0
    for nm in src.order:
        if nm in dst.cells: dup += 1; continue
        c = src.cells[nm]; dst.cells[nm] = c; dst.order.append(nm);
dst.stats['cells'] += 1
        for k in ('nldm', 'ccs', 'ecsm'):
            if c.flags[k]: dst.stats[k] += 1
        if not any(c.flags.values()): dst.stats['none'] += 1
    for k, v in src.types.items(): dst.types.setdefault(k, v)
    if dup: notes.append('%s: %d cell(s) already defined by an earlier
file were ignored' % (fname, dup))
    return dst

def merge_design(dst, src, fname, notes):
    """append the modules of src after those of dst; file text stays
byte-exact when exported in the same order"""
    first = True
    for mn in src.order:
        m = src.modules[mn]
        if first:
            m.lead = (dst.tail or '') + (m.lead or ''); dst.tail = '';
first = False
        if mn in dst.modules:
            notes.append('%s: module "%s" is already defined by an
earlier file (the first definition is used)' % (fname, mn))
            dst.errors.append((m.line, '%s: module "%s" defined more
than once' % (fname, mn)))
            dst.dups = getattr(dst, 'dups', []) + [(fname, mn)]
            continue
        m.srcfile = fname; dst.modules[mn] = m; dst.order.append(mn)
    if first: pass
    dst.tail = (dst.tail or '') + (src.tail or '')
    for ln, msg in src.errors: dst.errors.append((ln, '%s: %s' % (fname, msg)))
    return dst

# ---------------- connection expressions ----------------
def parse_expr_parts(raw):
    tt, tv, tp, te = lex(raw); n = len(tt); st = [0]
    def num(s):
        m = re.match(r"^(\d*)'([sS]?)([bBoOdDhH])\s*([0-9a-fA-F_xXzZ?]+)$", s)
        if m:
            size = int(m.group(1)) if m.group(1) else None; base =
m.group(3).lower(); digs = m.group(4).replace('_', '').lower()
            if base == 'd': bits = bin(int(digs))[2:] if digs.isdigit() else '0'
            else:
                w = {'b': 1, 'o': 3, 'h': 4}[base]; bits = ''
                for ch in digs:
                    if ch in 'xz?': bits += ('z' if ch == '?' else ch) * w
                    else: bits += format(int(ch, 16), '0%db' % w)
            if size is None: return ('c', bits, True)
            if len(bits) < size: bits = (bits[0] if bits[:1] in ('x',
'z') else '0') * (size - len(bits)) + bits
            else: bits = bits[len(bits) - size:]
            return ('c', bits, False)
        if re.match(r"^'[01xXzZ]$", s): return ('c', s[1].lower(), True)
        if re.match(r'^\d[\d_]*$', s): return ('c',
bin(int(s.replace('_', '')))[2:], True)
        return None
    def elem():
        if st[0] >= n: return None
        t = tt[st[0]]; v = tv[st[0]]
        if v == '{' and t == T_P: return concat()
        if t in (T_ID, T_ESC):
            st[0] += 1; part = ['n', v, None, None, None]   #
kind,name,idx,msb,lsb
            if st[0] < n and tv[st[0]] == '[':
                st[0] += 1
                if st[0] >= n or tt[st[0]] != T_NUM or not
tv[st[0]].isdigit(): return None
                a = int(tv[st[0]]); st[0] += 1; b = None
                if st[0] < n and tv[st[0]] == ':':
                    st[0] += 1
                    if st[0] >= n or tt[st[0]] != T_NUM or not
tv[st[0]].isdigit(): return None
                    b = int(tv[st[0]]); st[0] += 1
                if st[0] >= n or tv[st[0]] != ']': return None
                st[0] += 1
                if b is None: part[2] = a
                else: part[3] = a; part[4] = b
                if st[0] < n and tv[st[0]] == '[': return None
            return [tuple(part)]
        if t == T_NUM:
            st[0] += 1; c = num(v)
            return [c] if c else None
        return None
    def concat():
        st[0] += 1
        if st[0] < n and tt[st[0]] == T_NUM and st[0] + 1 < n and
tv[st[0] + 1] == '{' and tv[st[0]].isdigit():
            cnt = int(tv[st[0]]); st[0] += 2
            inner = lst()
            if inner is None or st[0] >= n or tv[st[0]] != '}': return None
            st[0] += 1
            if st[0] >= n or tv[st[0]] != '}': return None
            st[0] += 1
            return inner * cnt
        l = lst()
        if l is None or st[0] >= n or tv[st[0]] != '}': return None
        st[0] += 1; return l
    def lst():
        res = []
        while True:
            e = elem()
            if e is None: return None
            res.extend(e)
            if st[0] < n and tv[st[0]] == ',': st[0] += 1
            else: break
        return res
    if n == 0: return []
    r = elem()
    if r is None or st[0] < n: return None
    return r

def idents_in(raw):
    tt, tv, tp, te = lex(raw); out = []
    for i in range(len(tt)):
        if tt[i] in (T_ID, T_ESC) and tv[i] not in out: out.append(tv[i])
    return out

def rename_in_expr(raw, old, new):
    tt, tv, tp, te = lex(raw); out = []; last = 0
    for i in range(len(tt)):
        if tt[i] in (T_ID, T_ESC) and tv[i] == old:
            out.append(raw[last:tp[i]]); out.append(new + (' ' if
new.startswith('\\') else '')); last = te[i]
    out.append(raw[last:])
    return ''.join(out)

class Bits:
    __slots__ = ('bits', 'undeclared', 'oob', 'unsized', 'complex',
'empty', 'ids', 'unknown_width')
    def __init__(self):
        self.bits = []; self.undeclared = []; self.oob = [];
self.unsized = False; self.complex = False; self.empty = False;
self.ids = []; self.unknown_width = False

def subst_bit(m, expr, key, new):
    """expression `expr` with the single bit `key` replaced by the net
`new` (bus / concatenation connections); None if it cannot be
rebuilt"""
    b = bits_of(m, expr)
    if b.complex or b.empty or b.unsized: return None
    out = []
    for x in b.bits:
        if x[0] == 'c': out.append("1'b%s" % x[1])
        else: out.append(new if x[1] == key else x[1])
    if key not in [x[1] for x in b.bits if x[0] == 'n']: return None
    parts = []; i = 0; rx = re.compile(r'^(.+)\[(-?\d+)\]$')
    while i < len(out):
        mm = rx.match(out[i])
        if mm:
            base, hi = mm.group(1), int(mm.group(2)); j = i; lo = hi
            while j + 1 < len(out):
                m2 = rx.match(out[j + 1])
                if m2 and m2.group(1) == base and int(m2.group(2)) ==
lo - 1: lo -= 1; j += 1
                else: break
            parts.append('%s[%d]' % (base, hi) if hi == lo else
'%s[%d:%d]' % (base, hi, lo)); i = j + 1
        else: parts.append(out[i]); i += 1
    return parts[0] if len(parts) == 1 else '{%s}' % ', '.join(parts)

def bits_of(mod, raw):
    cache = mod._bc
    if cache is None: cache = mod._bc = {}
    r = cache.get(raw)
    if r is not None: return r
    res = Bits(); cache[raw] = res
    if raw == '' or raw is None: res.empty = True; return res
    parts = parse_expr_parts(raw)
    if parts is None:
        res.complex = True; res.ids = idents_in(raw); return res
    for pt in parts:
        if pt[0] == 'c':
            for ch in pt[1]: res.bits.append(('c', ch))
            if pt[2]: res.unsized = True
            continue
        _, nm, idx, msb, lsb = pt
        net = mod.nets.get(nm)
        if net is None:
            if nm not in res.undeclared: res.undeclared.append(nm)
            net = Net(nm)
        lo = min(net.msb, net.lsb); hi = max(net.msb, net.lsb)
        def ref(i): return ('n', '%s[%d]' % (nm, i) if net.vec else nm, nm, i)
        if idx is not None:
            if net.vec and lo == lo and (idx < lo or idx > hi):
res.oob.append('%s[%d]' % (nm, idx))
            res.bits.append(ref(idx))
        elif msb is not None:
            step = -1 if msb >= lsb else 1; i = msb
            while True:
                res.bits.append(ref(i))
                if i == lsb: break
                i += step
            if net.vec and lo == lo and (min(msb, lsb) < lo or
max(msb, lsb) > hi): res.oob.append('%s[%d:%d]' % (nm, msb, lsb))
        elif net.vec and net.msb != net.msb:
            res.complex = True; res.unknown_width = True
            if nm not in res.ids: res.ids.append(nm)
        elif net.vec:
            step = -1 if net.msb >= net.lsb else 1; i = net.msb
            while True:
                res.bits.append(ref(i))
                if i == net.lsb: break
                i += step
        else: res.bits.append(ref(0))
    return res

# ---------------- writer ----------------
def nmo(n): return n + ' ' if n.startswith('\\') else n
def exo(e): return e + ' ' if re.search(r'(^|[\s{,(\[])\\\S+$', e) else e
def range_str(n): return (n.raw_range or '[%d:%d]' % (n.msb, n.lsb))
if n.vec else ''
def port_decl(p, n):
    return p.dir + (' ' + p.type if p.type else '') + (' ' +
range_str(n) if n is not None and n.vec else '') + ' ' + nmo(p.name)

def write_inst(i):
    out = []
    head = '  ' + i.type + (' ' + i.params if i.params else '') + (' '
+ i.name if i.name else '') + (' ' + i.arr if i.arr else '') + ' ('
    if i.attrs: out.append('  ' + i.attrs)
    if i.prim or i.positional:
        out.append(head + ', '.join(exo(c.expr) for c in i.conns) +
');'); return out
    if not i.conns: out.append(head + ');'); return out
    out.append(head)
    for k, c in enumerate(i.conns):
        pin = '.*' if c.pin == '*' else '.' + c.pin + (' ' if
c.pin.startswith('\\') else '') + '(' + exo(c.expr) + ')'
        out.append('      ' + pin + (',' if k < len(i.conns) - 1 else ''))
    out.append('  );')
    return out

def module_lines(m, sink=None):
    """-> (lines, meta).  With sink(text) the lines are streamed out
in chunks instead (lines is None, meta is still complete)"""
    L = []; meta = []; st = [0]
    def add(s, inst=None):
        for x in str(s).split('\n'): L.append(x); meta.append(inst)
        if sink is not None and len(L) >= 40000:
            sink(('\n' if st[0] else '') + '\n'.join(L)); st[0] += 1; L.clear()
    hdr = 'module ' + m.name + (' ' + m.params if m.params else '')
    if m.ansi:
        if m.ports:
            add(hdr + ' (')
            for k, p in enumerate(m.ports): add('    ' + port_decl(p,
m.nets.get(p.name)) + (',' if k < len(m.ports) - 1 else ''))
            add(');')
        else: add(hdr + ' ();')
    else:
        add(hdr + ' (' + ', '.join(nmo(p.name) for p in m.ports) + ');')
        for p in m.ports: add('  ' + port_decl(p, m.nets.get(p.name)) + ';')
    for s in m.param_lines: add('  ' + s)
    is_port = {p.name for p in m.ports}; group = None
    def flush():
        nonlocal group
        if group: add('  ' + (group[1] or 'wire') + (' ' + group[2] if
group[2] else '') + ' ' + ', '.join(nmo(x) for x in group[3]) + ';');
group = None
    for nm in m.net_order:
        n = m.nets.get(nm)
        if n is None or nm in is_port: continue
        key = (n.type or 'wire') + '|' + range_str(n)
        if group and group[0] == key: group[3].append(nm)
        else: flush(); group = [key, n.type, range_str(n), [nm]]
    flush()
    for a in m.assigns: add('  assign ' + a.lhs + ' = ' + exo(a.rhs) + ';')
    if isinstance(m.insts, LazyInsts):
        for txt, i in m.insts.iter_out():
            if i is None: add('  ' + txt)
            else:
                for s in write_inst(i): add(s, i.name)
    else:
        for i in m.insts:
            for s in write_inst(i): add(s, i.name)
    for r in m.raw: add('  ' + r.lstrip())
    add('endmodule')
    if sink is not None:
        if L: sink(('\n' if st[0] else '') + '\n'.join(L))
        return None, meta
    return L, meta

def module_text(m): return '\n'.join(module_lines(m)[0]) + '\n'

def write_design(D, want_files=False):
    parts = []; lmap = LineMap(); nlc = 0; files = {} if want_files else None
    for mn in D.order:
        m = D.modules[mn]; lead = m.lead if m.lead is not None else '\n\n'
        parts.append(lead); nlc += lead.count('\n')
        if not m.dirty and m.src:
            body = m.src; n = body.count('\n') + 1
            lmap.add(nlc + 1, n, mn, 'exact', exact_items(m))
        else:
            L, meta = module_lines(m); body = '\n'.join(L);
lmap.add(nlc + 1, len(L), mn, 'list', meta)
        parts.append(body); nlc += body.count('\n')
        if files is not None: files[mn] = body + '\n'
    parts.append(D.tail or '')
    s = ''.join(parts)
    if not s.endswith('\n'): s += '\n'
    return s, lmap, files

def exact_items(m):
    """(starts, ends, names) of an unmodified module's instances,
compact for millions of them"""
    if isinstance(m.insts, LazyInsts): return m.insts.exact          #
computed on first lookup
    L = [i for i in m.insts if i.name]; st = array('i'); en =
array('i'); nm = []; last = -1; mono = True
    for i in L:
        if i.line < last: mono = False
        last = i.line; st.append(i.line); en.append(i.line2); nm.append(i.name)
    if mono: return st, en, nm
    items = sorted((i.line, i.line2, i.name) for i in L)
    return array('i', [x[0] for x in items]), array('i', [x[1] for x
in items]), [x[2] for x in items]

def write_design_to(D, f):
    """stream the design to an open text file without building it in
memory; -> LineMap"""
    lmap = LineMap(); nlc = 0; first = True
    for mn in D.order:
        m = D.modules[mn]; lead = m.lead if m.lead is not None else '\n\n'
        f.write(lead); nlc += lead.count('\n')
        if not m.dirty and m.src:
            body = m.src; n = body.count('\n') + 1
            lmap.add(nlc + 1, n, mn, 'exact', exact_items(m))
            f.write(body); nlc += n - 1
        else:
            _, meta = module_lines(m, f.write); lmap.add(nlc + 1,
len(meta), mn, 'list', meta); nlc += len(meta) - 1
    tail = D.tail or ''; f.write(tail)
    if not tail.endswith('\n'): f.write('\n')
    return lmap

# ---------------- model editing ----------------
def inst_type_set(m):
    L = m.insts
    return L.type_set() if isinstance(L, LazyInsts) else {i.type for i in L}
def insts_of_types(m, types):
    L = m.insts
    return L.of_types(types) if isinstance(L, LazyInsts) else [i for i
in L if i.type in types]
def type_counts(m):
    from collections import Counter
    L = m.insts
    return L.type_counts() if isinstance(L, LazyInsts) else
Counter(i.type for i in L)
def child_insts(D, m):
    """instances of m whose type is a module of the design"""
    L = m.insts
    if isinstance(L, LazyInsts):
        t = {x for x in L.type_set() if x in D.modules}
        return L.of_types(t) if t else []
    return [i for i in L if i.type in D.modules]

def parents_of(D, name):
    out = []
    for mn in D.order:
        for i in insts_of_types(D.modules[mn], {name}):
out.append((D.modules[mn], i))
    return out

def unique_name(taken, base):
    n = base; k = 1
    while taken(n): n = '%s_%d' % (base, k); k += 1
    return n

def add_instance(m, type_, base=None):
    nm = unique_name(lambda n: m.inst(n) is not None or n in m.nets,
base or ('U_' + re.sub(r'[^a-z0-9_]', '_', type_.lower())))
    i = Inst(nm, type_); i.prim = type_.lower() in PRIMS;
m.insts.append(i); jpres(m, i, len(m.insts) - 1, True); ix_set(m, i,
True); soft_touch(m, i); return i

def move_inst_to(m, inst, idx):
    """place an instance (just appended) at list position idx, so it
shows on the page being viewed"""
    L = m.insts
    if L and L[-1] is inst and 0 <= idx < len(L) - 1:
        if JRN is not None:
            for it in reversed(JRN.items):
                if isinstance(it, _PresItem) and it.inst is inst:
it.idx = idx; break
        L.pop(); L.insert(idx, inst); soft_touch(m, inst)

def remove_instance(m, name):
    if JRN is not None:
        L = m.insts
        if isinstance(L, LazyInsts):
            i = L.find(name); k = L.index_of(i) if i is not None else -1
            if k >= 0: jpres(m, i, k, False); del L[k]; soft_touch(m, i)
            return
        for k, i in enumerate(m.insts):
            if i.name == name: jpres(m, i, k, False); del m.insts[k];
ix_set(m, i, False); soft_touch(m, i); return
        return
    m.insts = [i for i in m.insts if i.name != name]; touch(m)

def set_conn(m, inst, pin, expr):
    jinst(inst); c = None
    for x in inst.conns:
        if x.pin == pin: c = x
    if expr is None: inst.conns = [x for x in inst.conns if x is not c]
    elif c: c.expr = expr
    else: inst.conns.append(Conn(pin, expr))
    soft_touch(m, inst)

def conn_expr(inst, pin):
    for c in inst.conns:
        if c.pin == pin: return c.expr
    return None

def rename_net(D, m, old, new):
    n = m.nets.get(old)
    if not n: return
    jnet(m, old); jnet(m, new); jlist(m, 'net_order'); jlist(m,
'ports'); jlist(m, 'assigns')
    for p in m.ports: jobj(p)
    for a in m.assigns: jobj(a)
    jobj(n)
    del m.nets[old]; n.name = new; m.nets[new] = n
    m.net_order = [new if x == old else x for x in m.net_order]
    for p in m.ports:
        if p.name == old: p.name = new
    for a in m.assigns: a.lhs = rename_in_expr(a.lhs, old, new); a.rhs
= rename_in_expr(a.rhs, old, new)
    for i in m.insts:
        if JRN is not None and any(old in c.expr for c in i.conns): jinst(i)
        for c in i.conns: c.expr = rename_in_expr(c.expr, old, new)
    if n.dir:
        for pm, pi in parents_of(D, m.name):
            jinst(pi)
            for c in pi.conns:
                if c.pin == old: c.pin = new
            touch(pm)
    touch(m)

def add_net(m, name, rng=None, type_=''):
    r = (rng[0], rng[1], '[%d:%d]' % (rng[0], rng[1])) if rng else None
    jnet(m, name); jlist(m, 'net_order')
    n = declare_net(m, name, r, type_, None); n.declared = True;
soft_touch(m); return n

def remove_net(m, name):
    n = m.nets.get(name)
    if not n or n.dir: return
    jnet(m, name); jlist(m, 'net_order'); jlist(m, 'assigns')
    del m.nets[name]; m.net_order = [x for x in m.net_order if x != name]
    for i in m.insts:
        for c in i.conns:
            if name in c.expr and name in idents_in(c.expr): jinst(i);
c.expr = ''
    m.assigns = [a for a in m.assigns if name not in idents_in(a.lhs)
and name not in idents_in(a.rhs)]
    touch(m)

def add_port(D, m, name, dir_, rng=None):
    jlist(m, 'ports'); jnet(m, name); jlist(m, 'net_order')
    m.ports.append(Port(name, dir_, ''))
    r = (rng[0], rng[1], '[%d:%d]' % (rng[0], rng[1])) if rng else None
    n = declare_net(m, name, r, '', dir_); n.dir = dir_; soft_touch(m)

def remove_port(D, m, name):
    jlist(m, 'ports'); jnet(m, name); jlist(m, 'net_order'); jlist(m, 'assigns')
    m.ports = [p for p in m.ports if p.name != name]
    m.nets.pop(name, None); m.net_order = [x for x in m.net_order if x != name]
    for i in m.insts:
        for c in i.conns:
            if name in c.expr and name in idents_in(c.expr): jinst(i);
c.expr = ''
    m.assigns = [a for a in m.assigns if name not in idents_in(a.lhs)
and name not in idents_in(a.rhs)]
    for pm, pi in parents_of(D, m.name):
        jinst(pi); pi.conns = [c for c in pi.conns if c.pin != name]; touch(pm)
    touch(m)

def set_net_shape(m, name, rng, dir_=None):
    n = m.nets.get(name)
    if not n: return
    jobj(n)
    for p in m.ports: jobj(p)
    if rng: n.msb, n.lsb, n.vec, n.raw_range = rng[0], rng[1], True,
'[%d:%d]' % (rng[0], rng[1])
    else: n.msb = n.lsb = 0; n.vec = False; n.raw_range = ''
    if dir_:
        n.dir = dir_
        for p in m.ports:
            if p.name == name: p.dir = dir_
    touch(m)

def rename_module(D, old, new):
    m = D.modules.pop(old); m.name = new; D.modules[new] = m
    D.order = [new if x == old else x for x in D.order]
    for mn in D.order:
        for i in insts_of_types(D.modules[mn], {old}): jinst(i);
i.type = new; touch(D.modules[mn])
    touch(m)

# ====================================================================
#  Resolution + sanity checks (Verilator-style) + simulation models
# ====================================================================
from collections import namedtuple
PinDef = namedtuple('PinDef', 'name dir width msb lsb tri')
VL_CODES = {'PINCONNECTEMPTY', 'PINMISSING', 'PINNOTFOUND',
'WIDTHTRUNC', 'WIDTHEXPAND', 'UNUSEDSIGNAL', 'UNDRIVEN',
            'IMPLICIT', 'MODMISSING', 'MULTITOP', 'REDEFINE',
'SELRANGE', 'SYNTAX'}

def resolve_type(D, lib, t):
    if t in D.modules: return ('module', D.modules[t])
    if lib and t in lib.cells: return ('cell', lib.cells[t])
    if t.lower() in PRIMS: return ('prim', None)
    return ('unknown', None)

def type_pins(rt, inst):
    k, o = rt
    if k == 'cell': return [PinDef(p.name, p.dir, p.width, p.msb,
p.lsb, bool(p.tri)) for p in o.pins]
    if k == 'module':
        out = []
        for p in o.ports:
            n = o.nets.get(p.name); out.append(PinDef(p.name, p.dir,
net_width(n), n.msb if n else 0, n.lsb if n else 0, False))
        return out
    if k == 'prim': return [PinDef(n, d, w, 0, 0, False) for n, d, w
in prim_pins(inst.type, len(inst.conns))]
    return []

def named_conns(inst, pins):
    if not inst.positional or inst.prim: return [(c.pin, c.expr) for c
in inst.conns]
    return [(pins[i].name if i < len(pins) else None, c.expr) for i, c
in enumerate(inst.conns)]

def find_tops(D):
    used = set()
    for mn in D.order:
        for t in inst_type_set(D.modules[mn]):
            if t in D.modules: used.add(t)
    return [m for m in D.order if m not in used]

def hier_depth(D, mn, memo=None, stack=()):
    if memo is None: memo = {}
    if mn in memo: return memo[mn]
    if mn in stack: return 0
    d = 0
    for t in {i.type for i in child_insts(D, D.modules[mn])}:
        d = max(d, 1 + hier_depth(D, t, memo, stack + (mn,)))
    memo[mn] = d
    return d

def fmt_bits(name, idxs):
    idxs = sorted(idxs, reverse=True); out = []; s = e = idxs[0]
    for k in range(1, len(idxs) + 1):
        if k < len(idxs) and idxs[k] == e - 1: e = idxs[k]; continue
        out.append('%s[%d]' % (name, s) if s == e else '%s[%d:%d]' %
(name, s, e))
        if k < len(idxs): s = e = idxs[k]
    return ', '.join(out)

class Issue:
    __slots__ = ('sev', 'code', 'cat', 'msg', 'mod', 'inst', 'net',
'src', 'vl', 'line', 'pin')
    def __init__(self, sev, code, cat, msg, mod='', inst='', net='',
src='builtin', pin='', line=0):
        self.sev = sev; self.code = code; self.cat = cat; self.msg =
msg; self.mod = mod; self.inst = inst
        self.net = net; self.src = src; self.vl = code in VL_CODES;
self.pin = pin; self.line = line

def bit_keys(n):
    """(key, idx) for every bit of net n, MSB first"""
    w = net_width(n)
    if not n.vec: return [(n.name, 0)]
    step = -1 if n.msb >= n.lsb else 1
    return [('%s[%d]' % (n.name, n.msb + step * i), n.msb + step * i)
for i in range(w)]

def run_checks(D, lib, top=None, progress=None, cancel=None):
    issues = []
    def add(*a, **k): issues.append(Issue(*a, **k))
    for line, msg in D.errors: add('error', 'SYNTAX', 'syntax', '%s
(line %d)' % (msg, line), line=line)
    tops = find_tops(D)
    if not top or top not in D.modules: top = tops[0] if len(tops) ==
1 else None
    if not D.order: add('error', 'NOTOP', 'hierarchy', 'No modules
found in the netlist.')
    if len(tops) > 1: add('warn', 'MULTITOP', 'hierarchy', 'Multiple
top-level modules (%s). Verilator needs --top-module to pick one.' %
', '.join(tops))
    if not tops and D.order: add('error', 'NOTOP', 'hierarchy', 'Every
module is instantiated by another one: the hierarchy has a cycle.')
    state = {}
    def dfs(mn, path):
        if state.get(mn) == 1: add('error', 'RECURSION', 'hierarchy',
'Recursive instantiation: ' + ' -> '.join(path + [mn]), mn); return
        if state.get(mn) == 2: return
        state[mn] = 1
        for t in sorted({i.type for i in child_insts(D,
D.modules[mn])}): dfs(t, path + [mn])
        state[mn] = 2
    for mn in D.order: dfs(mn, [])
    deps_cache = {}
    for i, mn in enumerate(D.order):
        if cancel and cancel(): raise JobCancelled()
        if progress: progress(i / max(1, len(D.order)), 'checking
module %s (%d of %d)' % (mn, i + 1, len(D.order)))
        _check_module(D, lib, D.modules[mn], add, deps_cache)
    used = {t for mn in D.order for t in inst_type_set(D.modules[mn])
if t not in D.modules and lib and t in lib.cells}
    if lib:
        for cn in sorted(used): lib_check(lib.cells[cn], add)
    return issues, top

def _check_module(D, lib, m, add, deps_cache):
    drivers = {}; loads = {}; touched = set(); seen_inst = set();
reported = set()
    def drv(k, o): drivers.setdefault(k, []).append(o)
    def ld(k, o): loads.setdefault(k, []).append(o)
    for n in m.dups: add('error', 'REDEFINE', 'structural', 'Net "%s"
is declared more than once.' % n, m.name, '', n)
    for p in m.ports:
        n = m.nets.get(p.name)
        if n is None or (n.vec and n.msb != n.msb): continue
        for k, _ in bit_keys(n):
            if p.dir in ('input', 'inout'): drv(k, ('port', p.dir == 'inout'))
            if p.dir in ('output', 'inout'): ld(k, ('port',))
    if m.raw:
        add('info', 'BEHAV', 'structural', '%d behavioral block%s
(always/initial/generate/function) kept verbatim but not analysed;
signals they mention count as driven and used.' % (len(m.raw), '' if
len(m.raw) == 1 else 's'), m.name)
        for r in m.raw: touched.update(idents_in(r))
    for inst in m.insts:
        label = inst.name or '(unnamed %s)' % inst.type
        if inst.name:
            if inst.name in seen_inst: add('error', 'REDEFINE',
'structural', 'Instance name "%s" is used more than once.' %
inst.name, m.name, inst.name)
            seen_inst.add(inst.name)
            if inst.name in m.nets: add('warn', 'NAMECLASH',
'structural', 'Instance "%s" has the same name as a net.' % inst.name,
m.name, inst.name)
        rt = resolve_type(D, lib, inst.type); pins = type_pins(rt,
inst); pmap = {p.name: p for p in pins}
        if rt[0] == 'unknown': add('error', 'MODMISSING',
'structural', 'Cannot find a module or library cell named "%s"
(instance %s).' % (inst.type, label), m.name, inst.name)
        if inst.arr: add('info', 'UNSUPPORTED', 'structural',
'Instance array %s is kept as written but not expanded in the
schematic.' % inst.arr, m.name, inst.name)
        conns = named_conns(inst, pins); connected = {}
        if inst.positional and not inst.prim and rt[0] != 'unknown'
and len(conns) > len(pins):
            add('error', 'PINNOTFOUND', 'structural', 'Instance %s has
%d positional connections but %s has only %d pins.' % (label,
len(conns), inst.type, len(pins)), m.name, inst.name)
        for pin, expr in conns:
            if pin == '*': continue
            pd = pmap.get(pin); eb = bits_of(m, expr)
            for u in eb.undeclared:
                if u not in reported:
                    reported.add(u); add('warn', 'IMPLICIT',
'structural', 'Net "%s" is used in a connection but never declared
(implicit wire).' % u, m.name, inst.name, u)
            for o in eb.oob: add('error', 'SELRANGE', 'structural',
'Bit/part select %s is outside the declared range.' % o, m.name,
inst.name)
            if rt[0] == 'unknown':
                for b in eb.bits:
                    if b[0] == 'n': touched.add(b[1])
                touched.update(eb.ids); continue
            if pd is None:
                if rt[0] == 'cell' and pin in rt[1].pg: continue
                if pin: add('error', 'PINNOTFOUND', 'structural',
'"%s" has no pin named "%s" (instance %s).' % (inst.type, pin, label),
m.name, inst.name, '', pin=pin)
                continue
            connected[pin] = not eb.empty
            if eb.empty:
                add('warn', 'PINCONNECTEMPTY', 'structural', 'Pin
%s.%s is connected to nothing: .%s().' % (label, pin, pin), m.name,
inst.name, pin=pin); continue
            if eb.complex:
                add('info', 'EXPRESSION', 'structural', 'Connection
%s.%s = "%s" is an expression; connectivity is approximated.' %
(label, pin, expr), m.name, inst.name, pin=pin)
                for idn in eb.ids:
                    n = m.nets.get(idn)
                    if n and not (n.vec and n.msb != n.msb):
                        for k, _ in bit_keys(n):
                            if pd.dir == 'input': ld(k, ('inst',
inst.name, pin))
                            else: touched.add(k)
                    else: touched.add(idn)
                continue
            if not eb.unsized and pd.width != len(eb.bits) and
pd.width == pd.width:
                add('warn', 'WIDTHTRUNC' if len(eb.bits) > pd.width
else 'WIDTHEXPAND', 'structural',
                    'Pin %s.%s is %d bit%s wide but its connection
"%s" is %d.' % (label, pin, pd.width, 's' if pd.width > 1 else '',
expr, len(eb.bits)), m.name, inst.name, pin=pin)
            nb = min(pd.width or 1, len(eb.bits))
            for q in range(nb):
                b = eb.bits[len(eb.bits) - 1 - q]
                if b[0] == 'c':
                    if pd.dir in ('output', 'inout'): add('error',
'CONSTOUT', 'logic', "Output pin %s.%s is tied to the constant 1'b%s."
% (label, pin, b[1]), m.name, inst.name, pin=pin)
                    continue
                info = ('inst', inst.name, pin, pd.tri or pd.dir == 'inout')
                if pd.dir in ('output', 'inout'): drv(b[1], info)
                if pd.dir in ('input', 'inout'): ld(b[1], info)
        if rt[0] == 'unknown': continue
        wild = any(p == '*' for p, e in conns)
        if wild: add('info', 'WILDCARD', 'structural', 'Instance %s
uses .* so unlisted pins connect implicitly by name; they are not
drawn.' % label, m.name, inst.name)
        for p in pins:
            if wild or p.dir in ('power', 'ground'): continue
            if p.name not in connected:
                if rt[0] == 'cell' and p.dir == 'input': add('error',
'PINMISSING', 'logic', 'Input pin %s.%s is not connected: it floats.'
% (label, p.name), m.name, inst.name, pin=p.name)
                else: add('warn', 'PINMISSING', 'structural', 'Pin
%s.%s of %s is not connected.' % (label, p.name, inst.type), m.name,
inst.name, pin=p.name)
        if rt[0] == 'cell':
            d = classify_cell(rt[1])
            if d['kind'] in ('seq', 'icg'):
                if d['kind'] == 'seq': cp = d['clk'][0] if d['clk'] else None
                else: cp = next((p.name for p in rt[1].pins if
p.attrs.get('clock_gate_clock_pin') == 'true'), None)
                if cp:
                    cc = next((e for p, e in conns if p == cp), None)
                    if not cc: add('error', 'CLKUNCONN', 'logic',
'Clock pin %s.%s is not connected.' % (label, cp), m.name, inst.name,
pin=cp)
                    else:
                        cb = bits_of(m, cc)
                        if not cb.complex and cb.bits and all(b[0] ==
'c' for b in cb.bits):
                            add('warn', 'CLKCONST', 'logic', 'Clock
pin %s.%s is tied to a constant, so the cell never clocks.' % (label,
cp), m.name, inst.name, pin=cp)
    for a in m.assigns:
        lb = bits_of(m, a.lhs); rb = bits_of(m, a.rhs)
        for u in lb.undeclared + rb.undeclared:
            if u not in reported: reported.add(u); add('warn',
'IMPLICIT', 'structural', 'Net "%s" is used in an assign but never
declared.' % u, m.name, '', u)
        if lb.complex: touched.update(lb.ids)
        else:
            for b in lb.bits:
                if b[0] == 'c': add('error', 'CONSTOUT', 'logic',
'assign target "%s" is a constant.' % a.lhs, m.name)
                else: drv(b[1], ('assign',))
        if rb.complex:
            for idn in rb.ids:
                n = m.nets.get(idn)
                if n and not (n.vec and n.msb != n.msb):
                    for k, _ in bit_keys(n): ld(k, ('assign',))
                else: touched.add(idn)
        else:
            for b in rb.bits:
                if b[0] == 'n': ld(b[1], ('assign',))
        if not lb.complex and not rb.complex and not rb.unsized and
len(lb.bits) != len(rb.bits):
            add('warn', 'WIDTHTRUNC' if len(rb.bits) > len(lb.bits)
else 'WIDTHEXPAND', 'structural', 'assign %s = %s: left side is %d
bits, right side is %d.' % (a.lhs, a.rhs, len(lb.bits), len(rb.bits)),
m.name)
    agg = {}
    for nm in m.net_order:
        n = m.nets.get(nm)
        if n is None or (n.vec and n.msb != n.msb) or n.type in
('supply0', 'supply1'): continue
        for k, bi in bit_keys(n):
            if k in touched or nm in touched: continue
            Dl = drivers.get(k, []); Ll = loads.get(k, []); is_out =
n.dir == 'output'
            if not Dl and (Ll or is_out): agg.setdefault(('UNDRIVEN',
nm), []).append(bi)
            elif not Dl and not Ll: agg.setdefault(('ORPHAN', nm),
[]).append(bi)
            elif Dl and not Ll and not is_out:
agg.setdefault(('UNUSED', nm), []).append(bi)
            if len(Dl) > 1:
                if any(d[0] == 'port' and not d[1] for d in Dl):
agg.setdefault(('DRIVENINPUT', nm), []).append(bi)
                elif not all((len(d) > 3 and d[3]) or (d[0] == 'port'
and d[1]) for d in Dl): agg.setdefault(('MULTIDRIVEN', nm),
[]).append(bi)
    for (kind, nm), idxs in agg.items():
        n = m.nets[nm]; sc = fmt_bits(nm, idxs) if n.vec else nm;
is_port = bool(n.dir)
        if kind == 'UNDRIVEN': add('warn', 'UNDRIVEN', 'logic', '%s
"%s" is never driven (it reads as X/0 in a 2-state simulator).' %
('Port' if is_port else 'Signal', sc), m.name, '', nm)
        elif kind == 'ORPHAN': add('warn', 'UNUSEDSIGNAL',
'structural', '%s "%s" is neither driven nor used.' % ('Port' if
is_port else 'Signal', sc), m.name, '', nm)
        elif kind == 'UNUSED': add('warn', 'UNUSEDSIGNAL',
'structural', '%s "%s" is declared but not used.' % ('Input port' if
n.dir == 'input' else 'Port' if is_port else 'Signal', sc), m.name,
'', nm)
        elif kind == 'MULTIDRIVEN': add('error', 'MULTIDRIVEN',
'logic', 'Signal "%s" has more than one driver.' % sc, m.name, '', nm)
        elif kind == 'DRIVENINPUT': add('error', 'DRIVENINPUT',
'logic', 'Input port "%s" is also driven from inside the module.' %
sc, m.name, '', nm)
    deps = mod_deps(D, lib, m, deps_cache, ())
    for lp in deps[1]: add('error', 'UNOPTFLAT', 'logic',
'Combinational loop: ' + ' -> '.join(lp) + '.', m.name,
lp[0].split('/')[0])

def lib_check(cell, add):
    pins = {p.name for p in cell.pins}; internal = set()
    for g in (cell.ff, cell.latch):
        if g: internal.update(g[0])
    def chk(expr, what):
        if not expr: return
        a = parse_func(expr)
        if a is None: add('error', 'LIBFUNC', 'library', 'Cell %s:
cannot parse %s "%s".' % (cell.name, what, expr)); return
        for v in ast_vars(a):
            if v not in pins and v not in internal: add('error',
'LIBFUNC', 'library', 'Cell %s: %s refers to "%s", which is not a pin
or internal state variable.' % (cell.name, what, v))
    for p in cell.pins:
        chk(p.func, 'function of pin ' + p.name); chk(p.tri,
'three_state of pin ' + p.name)
        if p.dir == 'output' and not p.func and not cell.memory and
not cell.statetable:
            add('info', 'LIBNOFUNC', 'library', 'Cell %s: output pin
%s has no function; it is modelled as a black box.' % (cell.name,
p.name))
    for g in (cell.ff, cell.latch):
        if g:
            for k in ('clocked_on', 'next_state', 'enable', 'data_in',
'clear', 'preset'):
                if g[1].get(k): chk(g[1][k], k)

def mod_deps(D, lib, m, cache, stack):
    if m.name in cache: return cache[m.name]
    if m.name in stack: return ({}, [])
    G = {}; edge_inst = {}
    def edge(a, b, inst):
        G.setdefault(a, []).append(b); G.setdefault(b, []);
edge_inst[(a, b)] = inst
    for inst in m.insts:
        rt = resolve_type(D, lib, inst.type)
        if rt[0] == 'unknown': continue
        pins = type_pins(rt, inst); pmap = {p.name: p for p in pins}; cb = {}
        for pin, expr in named_conns(inst, pins):
            if pin in pmap:
                e = bits_of(m, expr)
                if not e.complex and not e.empty: cb[pin] = e
        def bits_for(pn):
            e = cb.get(pn); pd = pmap.get(pn)
            if not e: return []
            nb = min(pd.width or 1, len(e.bits)); out = []
            for q in range(nb):
                b = e.bits[len(e.bits) - 1 - q]
                if b[0] == 'n': out.append((b[1], q))
            return out
        outs = [p for p in pins if p.dir in ('output', 'inout')]; ins
= [p for p in pins if p.dir == 'input']
        if rt[0] == 'cell':
            d = classify_cell(rt[1])
            if d['kind'] in ('seq', 'icg'): continue
            for o in outs:
                pc = rt[1].pin_map.get(o.name); ast =
parse_func(pc.func) if pc and pc.func else None
                dep = ast_vars(ast) if ast else None
                if pc and pc.tri:
                    ta = parse_func(pc.tri)
                    if ta: dep = (dep or []) + ast_vars(ta)
                for ip in ins:
                    if dep is not None and ip.name not in dep: continue
                    for ib in bits_for(ip.name):
                        for ob in bits_for(o.name): edge(ib[0], ob[0],
inst.name)
        elif rt[0] == 'prim':
            for o in outs:
                for ip in ins:
                    for ib in bits_for(ip.name):
                        for ob in bits_for(o.name): edge(ib[0], ob[0],
inst.name)
        else:
            sub = mod_deps(D, lib, rt[1], cache, stack + (m.name,))
            for key in sub[0]:
                ib_s, ob_s = key
                mi = re.match(r'^(.*)\[(-?\d+)\]$', ib_s); mo =
re.match(r'^(.*)\[(-?\d+)\]$', ob_s)
                ibase, iidx = (mi.group(1), int(mi.group(2))) if mi
else (ib_s, None)
                obase, oidx = (mo.group(1), int(mo.group(2))) if mo
else (ob_s, None)
                if ibase not in pmap or obase not in pmap: continue
                def lsb_index(net, idx):
                    if idx is None or not net.vec: return 0
                    return abs(idx - net.lsb)
                ni = rt[1].nets.get(ibase); no = rt[1].nets.get(obase)
                ii = lsb_index(ni, iidx); oi = lsb_index(no, oidx)
                for a in [x for x in bits_for(ibase) if x[1] == ii]:
                    for b in [x for x in bits_for(obase) if x[1] ==
oi]: edge(a[0], b[0], inst.name)
    for a in m.assigns:
        lb = bits_of(m, a.lhs); rb = bits_of(m, a.rhs)
        if lb.complex: continue
        if rb.complex:
            for idn in rb.ids:
                n = m.nets.get(idn)
                if n and not (n.vec and n.msb != n.msb):
                    for k, _ in bit_keys(n):
                        for l in lb.bits:
                            if l[0] == 'n': edge(k, l[1], 'assign ' + a.lhs)
        else:
            for i in range(min(len(lb.bits), len(rb.bits))):
                l = lb.bits[len(lb.bits) - 1 - i]; r =
rb.bits[len(rb.bits) - 1 - i]
                if l[0] == 'n' and r[0] == 'n': edge(r[1], l[1],
'assign ' + a.lhs)
    # Tarjan SCC (iterative)
    idx = {}; low = {}; on = set(); stk = []; cnt = 0; loops = []
    for root in list(G):
        if root in idx: continue
        work = [[root, 0]]; idx[root] = low[root] = cnt; cnt += 1;
stk.append(root); on.add(root)
        while work:
            fr = work[-1]; v = fr[0]; nb = G[v]
            if fr[1] < len(nb):
                w = nb[fr[1]]; fr[1] += 1
                if w not in idx: idx[w] = low[w] = cnt; cnt += 1;
stk.append(w); on.add(w); work.append([w, 0])
                elif w in on: low[v] = min(low[v], idx[w])
            else:
                work.pop()
                if work: pv = work[-1][0]; low[pv] = min(low[pv], low[v])
                if low[v] == idx[v]:
                    comp = []
                    while True:
                        x = stk.pop(); on.discard(x); comp.append(x)
                        if x == v: break
                    if len(comp) > 1 or v in G[v]:
                        names = []
                        for a in comp:
                            for b in comp:
                                ii = edge_inst.get((a, b))
                                if ii and ii not in names: names.append(ii)
                        loops.append(names[:8] + (['...'] if
len(names) > 8 else []))
    io = {}; in_ports = []; out_set = set()
    for p in m.ports:
        n = m.nets.get(p.name)
        if not n or (n.vec and n.msb != n.msb): continue
        for k, _ in bit_keys(n):
            if p.dir in ('input', 'inout'): in_ports.append(k)
            if p.dir in ('output', 'inout'): out_set.add(k)
    for s in in_ports:
        seen = {s}; q = [s]
        while q:
            v = q.pop()
            for w in G.get(v, []):
                if w not in seen: seen.add(w); q.append(w)
        for k in seen:
            if k != s and k in out_set: io[(s, k)] = 1
    cache[m.name] = (io, loops)
    return cache[m.name]

def parse_verilator_log(text):
    out = []
    for l in text.splitlines():
        m = re.match(r'^%(Warning|Error)(?:-([A-Za-z0-9_]+))?:\s+(.+?):(\d+):(\d+):\s*(.*)$',
l)
        if m:
            out.append(Issue('error' if m.group(1) == 'Error' else
'warn', m.group(2) or 'ERROR', 'verilator', m.group(6),
src='verilator', line=int(m.group(4))))
            out[-1].net = m.group(3); continue     # net field
temporarily holds the file name
        m = re.match(r'^%(Warning|Error)(?:-([A-Za-z0-9_]+))?:\s*(.*)$', l)
        if m and not m.group(3).startswith('Exiting due to'):
            out.append(Issue('error' if m.group(1) == 'Error' else
'warn', m.group(2) or 'ERROR', 'verilator', m.group(3),
src='verilator'))
    return out

# ---------------- simulation models ----------------
def _port_decl(p):
    d = 'output' if p.dir == 'output' else 'inout' if p.dir == 'inout'
else 'input'
    return d + (' [%d:%d]' % (p.msb, p.lsb) if p.width > 1 or p.is_bus
else '') + ' ' + p.name

def cell_model(cell):
    ports = [p for p in cell.pins if p.dir in ('input', 'output', 'inout')]
    L = ['module %s (%s);' % (cell.name, ', '.join(_port_decl(p) for p
in ports))]
    outs = [p for p in ports if p.dir in ('output', 'inout')]
    def ex(s):
        a = parse_func(s); return ast_to_verilog(a) if a else None
    g = cell.ff or cell.latch
    if g:
        args, A = g; iq = args[0]; iqn = args[1] if len(args) > 1 else None
        L.append('  reg %s;' % iq)
        if iqn: L.append('  wire %s = ~%s;' % (iqn, iq))
        clr = ex(A['clear']) if A.get('clear') else None; pre =
ex(A['preset']) if A.get('preset') else None
        if cell.ff:
            cl = lit_of(parse_func(A.get('clocked_on')))
            if cl: evs = [('negedge ' if cl[1] else 'posedge ') + cl[0]]
            else:
                L.append("  wire clk_i = %s;" %
(ex(A.get('clocked_on')) or "1'b0")); evs = ['posedge clk_i']
            for nm, e in (('clear', A.get('clear')), ('preset',
A.get('preset'))):
                if not e: continue
                l = lit_of(parse_func(e))
                if l: evs.append(('negedge ' if l[1] else 'posedge ') + l[0])
                else: L.append('  wire %s_i = %s;' % (nm, ex(e)));
evs.append('posedge %s_i' % nm)
            L.append('  always @(%s) begin' % ' or '.join(evs)); first = True
            if clr: L.append("    if (%s) %s <= 1'b0;" % (clr, iq));
first = False
            if pre: L.append("    %s (%s) %s <= 1'b1;" % ('if' if
first else 'else if', pre, iq)); first = False
            L.append('    %s%s <= %s;' % ('' if first else 'else ',
iq, ex(A.get('next_state')) or "1'b0")); L.append('  end')
        else:
            L.append('  always @* begin'); f2 = True
            if clr: L.append("    if (%s) %s = 1'b0;" % (clr, iq)); f2 = False
            if pre: L.append("    %s (%s) %s = 1'b1;" % ('if' if f2
else 'else if', pre, iq)); f2 = False
            L.append('    %s (%s) %s = %s;' % ('if' if f2 else 'else
if', ex(A.get('enable')) or "1'b0", iq, ex(A.get('data_in')) or
"1'b0")); L.append('  end')
        for p in outs: L.append('  assign %s = %s;' % (p.name,
(ex(p.func) if p.func else None) or "1'b0"))
    else:
        for p in outs:
            e = ex(p.func) if p.func else None; w = p.width if p.width
> 1 else 1
            if e and p.tri: L.append("  assign %s = (%s) ? 1'bz : %s;"
% (p.name, ex(p.tri), e))
            elif e: L.append('  assign %s = %s;' % (p.name, e))
            else: L.append("  assign %s = %s; // no Liberty function:
modelled as constant 0" % (p.name, '{%d{1\'b0}}' % w if w > 1 else
"1'b0"))
    L.append('endmodule')
    return '\n'.join(L)

CELL_LINT_OFF = ['DECLFILENAME', 'UNUSEDSIGNAL', 'UNUSEDPARAM',
'UNDRIVEN', 'LATCH', 'COMBDLY', 'MULTIDRIVEN', 'WIDTH',
'PINCONNECTEMPTY', 'SYNCASYNCNET', 'UNOPTFLAT']
def gen_cell_models(lib, names):
    h = '// Simulation models generated from the Liberty "function",
"ff", "latch" and "three_state" attributes.\n// Timing (NLDM / CCS /
ECSM) is not modelled. Regenerate from the Liberty file instead of
editing.\n'
    h += ''.join('// verilator lint_off %s\n' % c for c in CELL_LINT_OFF)
    return h + '\n' + '\n\n'.join(cell_model(lib.cells[n]) for n in
names) + '\n\n' + ''.join('// verilator lint_on %s\n' % c for c in
CELL_LINT_OFF)

def used_cell_names(D, lib):
    s = set()
    for mn in D.order:
        for t in inst_type_set(D.modules[mn]):
            if t not in D.modules and lib and t in lib.cells: s.add(t)
    return sorted(s)

def gen_verilator_files(D, lib, top, cycles=64):
    sh = ('#!/bin/sh\n# Verilator sanity run generated by Netlist
Studio.\n# Usage: ./run_verilator.sh [top_module]
[--sim]\nTOP=${1:-%s}\nHERE=$(cd "$(dirname "$0")" && pwd)\ncd "$HERE"
|| exit 1\nCELLS=""\n[ -f cells_sim.v ] && CELLS="cells_sim.v"\n'
          'echo "== 1/2  Verilator lint (-Wall) =="\nverilator
--lint-only -Wall -Wno-DECLFILENAME --top-module "$TOP" design.v
$CELLS > verilator_lint.log 2>&1\nRC=$?\ncat verilator_lint.log\necho
"lint exit code: $RC"\n'
          'if [ "$2" = "--sim" ] || [ "$1" = "--sim" ]; then\n  echo
"== 2/2  build + random-stimulus smoke simulation =="\n  rm -rf
obj_dir\n'
          '  verilator --cc --exe --build -Wno-fatal -Wno-lint
-Wno-style --top-module "$TOP" design.v $CELLS tb_main.cpp -Mdir
obj_dir -o sim > sim_build.log 2>&1 || { cat sim_build.log; echo
"simulation build failed"; exit 1; }\n  ./obj_dir/sim\nfi\nexit
$RC\n') % top
    cpp = ''
    m = D.modules.get(top)
    if m:
        drive = []; clk = []; rst = []; outs = []
        for p in m.ports:
            n = m.nets.get(p.name); w = net_width(n)
            if not isId(p.name) or w > 64 or w != w: continue
            if p.dir in ('input', 'inout'):
                if re.search(r'(^|_)(clk|clock|ck|clki)(_|$)', p.name,
re.I): clk.append(p.name)
                elif
re.search(r'(^|_)(rst|reset|resetn|rstn|nrst|nreset)(_|$|n$)', p.name,
re.I):
                    rst.append((p.name, bool(re.search(r'(n|_n|b)$',
p.name, re.I) or re.match(r'^n', p.name, re.I))))
                else: drive.append((p.name, w))
            if p.dir in ('output', 'inout'): outs.append(p.name)
        c = '// Random-stimulus smoke simulation for Verilator.
Generated by Netlist Studio.\n#include "V%s.h"\n#include
"verilated.h"\n#include <cstdint>\n#include <cstdio>\n#include
<cstdlib>\n\nint main(int argc, char** argv) {\n
Verilated::commandArgs(argc, argv);\n  V%s* top = new V%s;\n  uint64_t
chk = 1469598103934665603ULL;\n  srand(1);\n  const int CYCLES = %d;\n
 for (int cyc = 0; cyc < CYCLES; cyc++) {\n' % (top, top, top, cycles)
        for n, w in drive: c += '    top->%s = ((uint64_t)rand() ^
((uint64_t)rand() << 31))%s;\n' % (n, '' if w >= 64 else ' & 0x%xULL'
% ((1 << w) - 1))
        for n, low in rst: c += '    top->%s = (cyc < 4) ? %d : %d;\n'
% (n, 0 if low else 1, 1 if low else 0)
        if clk:
            c += ''.join('    top->%s = 0;\n' % n for n in clk) + '
top->eval();\n' + ''.join('    top->%s = 1;\n' % n for n in clk) + '
 top->eval();\n'
        else: c += '    top->eval();\n'
        for n in outs: c += '    chk = (chk ^ (uint64_t)top->%s) *
1099511628211ULL;\n' % n
        cpp = c + '  }\n  printf("simulated %d cycles, output checksum
= %016llx\\n", CYCLES, (unsigned long long)chk);\n  top->final();\n
delete top;\n  return 0;\n}\n'
    readme = ('Netlist Studio
export\n=====================\n\ndesign.v           the netlist with
your ECO edits; hierarchy and unmodified modules are written as
loaded.\ncells_sim.v        simulation models generated from the
Liberty functions (timing is not modelled).\nrun_verilator.sh   lint
(-Wall) and, with --sim, a random-stimulus smoke
simulation.\ntb_main.cpp        the C++ harness used by --sim.\n\nRun:
  sh run_verilator.sh %s          (lint only)\n       sh
run_verilator.sh %s --sim    (lint + build + simulate)\n') % (top,
top)
    return sh, cpp, readme

def isId(s): return re.match(r'^[A-Za-z_][A-Za-z0-9_$]*$', s) is not None
def isEscId(s): return re.match(r'^\\\S+$', s) is not None

# ====================================================================
#  ECO engine: connect pins anywhere in the hierarchy
# ====================================================================
SIMPLE_SIG = re.compile(r'^(\\\S+|[A-Za-z_][A-Za-z0-9_$]*)\s*(\[\s*-?\d+\s*(?::\s*-?\d+\s*)?\])?$')
HPin = namedtuple('HPin', 'path inst pin')       # path: instance
names from the root down to the container module

class EcoError(Exception):
    pass

def sanitize(s): return re.sub(r'[^A-Za-z0-9_]', '_', s).strip('_') or 'n'

def hpin_str(hp):
    if hasattr(hp, 'expr'): return '/' + '/'.join(list(hp.path) + [hp.expr])
    return '/' + '/'.join(list(hp.path) + ([hp.inst] if hp.inst else
[]) + [hp.pin])

def chain_modules(D, root, path):
    mods = [D.modules[root]]
    for nm in path:
        inst = mods[-1].inst(nm)
        if inst is None or inst.type not in D.modules: raise
EcoError('Path /%s does not lead to a module.' % '/'.join(path))
        mods.append(D.modules[inst.type])
    return mods

def affected_modules(D, root, *hps):
    names = set()
    for hp in hps:
        for m in chain_modules(D, root, hp.path):
            names.add(m.name)
            for pm, pi in parents_of(D, m.name): names.add(pm.name)
    return sorted(names)

def hpin_info(D, lib, root, hp):
    """-> (role, width, current_expr) ; role: drv | load | io"""
    m = chain_modules(D, root, hp.path)[-1]
    if hp.inst is None:
        p = next((q for q in m.ports if q.name == hp.pin), None)
        if p is None: raise EcoError('Port %s not found in %s.' %
(hp.pin, m.name))
        return ('drv' if p.dir == 'input' else 'load' if p.dir ==
'output' else 'io'), net_width(m.nets.get(p.name)), p.name
    inst = m.inst(hp.inst)
    if inst is None: raise EcoError('Instance %s not found in %s.' %
(hp.inst, m.name))
    rt = resolve_type(D, lib, inst.type); pins = type_pins(rt, inst)
    pd = next((p for p in pins if p.name == hp.pin), None)
    if pd is None: raise EcoError('%s has no pin %s.' % (inst.type, hp.pin))
    expr = next((e for p, e in named_conns(inst, pins) if p == hp.pin), None)
    return ('drv' if pd.dir == 'output' else 'load' if pd.dir ==
'input' else 'io'), pd.width, expr

def ensure_named(D, lib, inst):
    if inst.positional and not inst.prim:
        jinst(inst)
        pins = type_pins(resolve_type(D, lib, inst.type), inst)
        for i, c in enumerate(inst.conns): c.pin = pins[i].name if i <
len(pins) else 'p%d' % i
        inst.positional = False

def eco_new_net(D, m, hp, width, style='plain'):
    while True:
        D.eco_net_idx += 1
        base = hpin_str(hp)
        nm = (sanitize(base) if style == 'plain' else '\\' + base) +
'_eco_net_%d' % D.eco_net_idx
        if nm not in m.nets and m.inst(nm) is None: break
    add_net(m, nm, (width - 1, 0) if width > 1 else None)
    return nm

def eco_new_port(D, m, dir_, width):
    while True:
        D.eco_port_idx += 1
        nm = 'ECO_%s_hier_port_%d' % (sanitize(m.name), D.eco_port_idx)
        if nm not in m.nets and m.inst(nm) is None: break
    add_port(D, m, nm, dir_, (width - 1, 0) if width > 1 else None)
    return nm

def _uniquify(D, parent, inst, log, created):
    old = D.modules[inst.type]
    k = 1
    while True:
        nn = '%s_eco_%d' % (old.name, k)
        if nn not in D.modules: break
        k += 1
    new = copy.deepcopy(old); new.name = nn; new.dirty = True;
new.lead = '\n\n'; new.src = ''; new.layout = dict(old.layout or {})
    D.modules[nn] = new
    D.order.insert(D.order.index(old.name) + 1, nn)
    jinst(inst); inst.type = nn; soft_touch(parent, inst); created.append(nn)
    log.append('  uniquify module %s -> %s (for instance %s)' %
(old.name, nn, inst.name))
    return new

def _walk(D, root, path, from_k, ask, log, created):
    """Return modules along `path`, uniquifying shared modules below
index from_k when the user agrees."""
    mods = [D.modules[root]]
    for j, nm in enumerate(path):
        parent = mods[-1]; inst = parent.inst(nm)
        if inst is None or inst.type not in D.modules: raise
EcoError('Path /%s does not lead to a module.' % '/'.join(path[:j +
1]))
        mod = D.modules[inst.type]
        if j >= from_k:
            n_inst = len(parents_of(D, mod.name))
            if n_inst > 1 and ask:
                c = ask(mod.name, n_inst)
                if c == 'cancel': raise EcoError('Cancelled.')
                if c == 'uniquify': mod = _uniquify(D, parent, inst,
log, created)
        mods.append(mod)
    return mods

HNet = namedtuple('HNet', 'path expr')           # a wire (net or
bit-select) inside the module reached by `path`

def hstr(x):
    if isinstance(x, HNet): return '/' + '/'.join(list(x.path) + [x.expr])
    return hpin_str(x)

def net_has_driver(D, lib, m, base):
    for p in m.ports:
        if p.name == base and p.dir in ('input', 'inout'): return True
    for a in m.assigns:
        if base in idents_in(a.lhs): return True
    for inst in m.insts:
        rt = resolve_type(D, lib, inst.type); pins = type_pins(rt,
inst); pm = {p.name: p for p in pins}
        for pin, expr in named_conns(inst, pins):
            if expr and base in expr and pin in pm and pm[pin].dir in
('output', 'inout') and base in idents_in(expr): return True
    return False

def _ep(D, lib, root, x):
    if isinstance(x, HNet):
        m = chain_modules(D, root, x.path)[-1]; sm =
SIMPLE_SIG.match(x.expr or '')
        if not sm: raise EcoError('Only a named wire (or a bit of it)
can be used as a connection end.')
        if sm.group(1) not in m.nets: raise EcoError('Wire %s is not
declared in module %s.' % (sm.group(1), m.name))
        return {'kind': 'net', 'hp': x, 'path': x.path, 'role': 'net',
'width': sig_width(m, x.expr), 'expr': x.expr}
    role, w, expr = hpin_info(D, lib, root, x)
    return {'kind': 'pin' if x.inst else 'port', 'hp': x, 'path':
x.path, 'role': role, 'width': w, 'expr': expr}

def eco_connect(D, lib, root, a, b, ask=None, style='plain', ask_bit=None):
    """Connect two hierarchical endpoints. An endpoint is a pin, a
module port or a wire (HNet).
    Existing wires are reused. A new net is only created when the
connection needs one, i.e. when
    neither end already provides a wire in the lowest common module.
    Returns (log_lines, created_module_names)."""
    if isinstance(a, HNet) and isinstance(b, HNet): raise
EcoError('Select a pin as one end of the connection; two wires cannot
be joined this way.')
    ea = _ep(D, lib, root, a); eb = _ep(D, lib, root, b)
    if ea['kind'] == 'net' or eb['kind'] == 'net':
        pin_ep, net_ep = (eb, ea) if ea['kind'] == 'net' else (ea, eb)
        es, ed = (pin_ep, net_ep) if pin_ep['role'] == 'drv' else
(net_ep, pin_ep)
    else:
        ra, rb = ea['role'], eb['role']
        if ra == 'drv' and rb != 'drv': es, ed = ea, eb
        elif rb == 'drv' and ra != 'drv': es, ed = eb, ea
        elif ra == 'drv' and rb == 'drv': raise EcoError('Both pins
are drivers (%s and %s). Connecting them would create a multi-driven
net.' % (hstr(a), hstr(b)))
        else: es, ed = ea, eb
    src, dst = es['hp'], ed['hp']; log = ['ECO: connect %s -> %s' %
(hstr(src), hstr(dst))]; created = []
    CS = list(es['path']); CD = list(ed['path']); k = 0
    while k < len(CS) and k < len(CD) and CS[k] == CD[k]: k += 1
    Ms = _walk(D, root, CS, k, ask, log, created); Md = _walk(D, root,
CD, k, ask, log, created)
    lca = Ms[k]; msrc = Ms[-1]; mdst = Md[-1]
    # ---- the signal at the source end
    sexpr = None; src_complex = None; src_inst = None
    if es['kind'] == 'net': sexpr = es['expr']
    elif es['kind'] == 'port': sexpr = src.pin
    else:
        src_inst = msrc.inst(src.inst); ensure_named(D, lib,
src_inst); cur = conn_expr(src_inst, src.pin)
        if cur and SIMPLE_SIG.match(cur): sexpr = cur
        elif cur: src_complex = cur
    sw = es['width']
    if sexpr is not None: sw = sig_width(msrc, sexpr)
    # same module, wire as destination: the pin simply moves onto that wire
    if ed['kind'] == 'net' and es['kind'] == 'pin' and len(CS) == k
and len(CD) == k:
        old = conn_expr(src_inst, src.pin); set_conn(msrc, src_inst,
src.pin, ed['expr'])
        log.append('  %s.%s %s -> existing wire %s (no new net)' %
(src.inst, src.pin, ('was ' + old) if old else 'was open',
ed['expr']))
        D.eco_log.extend(log); return log, created
    dpin_inst = None; dcur = None
    if ed['kind'] == 'pin':
        dpin_inst = mdst.inst(dst.inst); ensure_named(D, lib,
dpin_inst); dcur = conn_expr(dpin_inst, dst.pin)
    wd = ed['width']
    if ask_bit and ed['kind'] == 'pin' and wd == 1 and sw > 1 and
sexpr is not None:
        sm2 = SIMPLE_SIG.match(sexpr); n2 =
msrc.nets.get(sm2.group(1)) if sm2 and not sm2.group(2) else None
        if n2 is not None and n2.vec:
            idx = ask_bit(sm2.group(1), min(n2.msb, n2.lsb),
max(n2.msb, n2.lsb))
            if idx is None: raise EcoError('Cancelled.')
            sexpr = '%s[%d]' % (sm2.group(1), idx); sw = 1;
log.append('  using bit %s of the %d-bit net' % (sexpr,
net_width(n2)))
    def drive_expr(): return sexpr if sexpr is not None else src_complex
    def free_net_of_dst():
        """a simple, driverless wire already on the destination pin:
it can take the source instead of a new net"""
        if ed['kind'] == 'pin' and len(CD) == k and dcur:
            sm = SIMPLE_SIG.match(dcur)
            if sm and not sm.group(2) and sm.group(1) in mdst.nets and
not net_has_driver(D, lib, mdst, sm.group(1)): return dcur
        return None
    if len(CS) == k:                                  # the source
lives in the lowest common module
        if sexpr is None:
            reuse = free_net_of_dst() if not src_complex else None
            if reuse:
                nn = reuse; set_conn(lca, src_inst, src.pin, nn);
log.append('  %s.%s joins existing net %s (no new net)' % (src.inst,
src.pin, nn))
            else:
                nn = eco_new_net(D, lca, src, sw, style)
                if src_complex: jlist(lca, 'assigns');
lca.assigns.append(Assign(nn, src_complex)); touch(lca); log.append('
new net %s in %s = %s' % (nn, lca.name, src_complex))
                else: set_conn(lca, src_inst, src.pin, nn);
log.append('  new net %s in module %s on %s' % (nn, lca.name,
hstr(src)))
            sexpr = nn
        else: log.append('  reusing existing net %s' % sexpr)
        N = sexpr
    else:                                             # source is
deeper: ports out of its modules, one wire in the lowest common module
        ports_s = [eco_new_port(D, Ms[j + 1], 'output', sw) for j in
range(k, len(CS))]
        for j in range(k, len(CS)): log.append('  add output port %s%s
to module %s' % (ports_s[j - k], ' [%d:0]' % (sw - 1) if sw > 1 else
'', Ms[j + 1].name))
        deep = ports_s[-1]
        if es['kind'] != 'pin': jlist(msrc, 'assigns');
msrc.assigns.append(Assign(deep, sexpr)); touch(msrc)
        elif sexpr is None and not src_complex: set_conn(msrc,
src_inst, src.pin, deep)
        else: jlist(msrc, 'assigns'); msrc.assigns.append(Assign(deep,
drive_expr())); touch(msrc)
        N = None
        if len(CD) == k:
            if ed['kind'] == 'net': N = ed['expr']
            elif ed['kind'] == 'port' and not net_has_driver(D, lib,
mdst, dst.pin): N = dst.pin
            else:
                N = free_net_of_dst()
            if N: log.append('  reusing existing net %s in module %s
(no new net)' % (N, lca.name))
        if N is None:
            N = eco_new_net(D, lca, src, sw, style); log.append('  new
net %s in module %s' % (N, lca.name))
        for j in range(k, len(CS)):
            inst = Ms[j].inst(CS[j]); ensure_named(D, lib, inst);
set_conn(Ms[j], inst, ports_s[j - k], N if j == k else ports_s[j - 1 -
k])
    # ---- the destination end
    if len(CD) == k: final = N
    else:
        ports_d = [eco_new_port(D, Md[j + 1], 'input', sw) for j in
range(k, len(CD))]
        for j in range(k, len(CD)): log.append('  add input port %s%s
to module %s' % (ports_d[j - k], ' [%d:0]' % (sw - 1) if sw > 1 else
'', Md[j + 1].name))
        for j in range(k, len(CD)):
            inst = Md[j].inst(CD[j]); ensure_named(D, lib, inst);
set_conn(Md[j], inst, ports_d[j - k], N if j == k else ports_d[j - 1 -
k])
        final = ports_d[-1]
    if ed['kind'] == 'port':
        if final != dst.pin:
            jlist(mdst, 'assigns'); mdst.assigns = [x for x in
mdst.assigns if x.lhs.strip() != dst.pin];
mdst.assigns.append(Assign(dst.pin, final)); touch(mdst)
    elif ed['kind'] == 'net':
        if final != ed['expr']: jlist(mdst, 'assigns');
mdst.assigns.append(Assign(ed['expr'], final)); touch(mdst);
log.append('  wire %s is now driven by %s' % (ed['expr'], final))
    else:
        if dcur != final:
            set_conn(mdst, dpin_inst, dst.pin, final); log.append('
%s.%s %s -> %s' % (dst.inst, dst.pin, ('was ' + dcur) if dcur else
'was open', final))
    if ed['kind'] == 'pin' and wd != sw and wd == wd: log.append('
warning: destination pin is %d bit%s wide but the source is %d' % (wd,
'' if wd == 1 else 's', sw))
    D.eco_log.extend(log)
    return log, created

def eco_disconnect(D, lib, root, hp):
    role, w, expr = hpin_info(D, lib, root, hp)
    m = chain_modules(D, root, hp.path)[-1]
    if hp.inst is None: raise EcoError('A module port cannot be
disconnected from itself.')
    inst = m.inst(hp.inst); ensure_named(D, lib, inst)
    set_conn(m, inst, hp.pin, '' if inst.prim else None)
    log = ['ECO: disconnect %s (was %s)' % (hpin_str(hp), expr or
'open')]; D.eco_log.extend(log); return log

def eco_tie(D, lib, root, hp, val):
    role, w, expr = hpin_info(D, lib, root, hp)
    m = chain_modules(D, root, hp.path)[-1]
    if hp.inst is None or role == 'drv': raise EcoError('Only input
pins of instances can be tied to a constant.')
    inst = m.inst(hp.inst); ensure_named(D, lib, inst)
    lit = ("%d'b%s" % (w, str(val) * w)) if w > 1 else "1'b%d" % val
    set_conn(m, inst, hp.pin, lit)
    log = ['ECO: tie %s to %s (was %s)' % (hpin_str(hp), lit, expr or
'open')]; D.eco_log.extend(log); return log


# ====================================================================
#  Regexp object search across the hierarchy
# ====================================================================
Found = namedtuple('Found', 'kind path name sub')     # kind: inst pin
net port topport

def hier_walk(D, root, start_path=(), recurse=True):
    m = chain_modules(D, root, start_path)[-1]; stack =
[(tuple(start_path), m, (m.name,))]
    while stack:
        path, m, types = stack.pop()
        yield path, m
        if not recurse: continue
        for inst in reversed(child_insts(D, m)):
            t = inst.type
            if t not in types and len(path) < 64: stack.append((path +
(inst.name,), D.modules[t], types + (t,)))

def found_str(f):
    parts = list(f.path) + [f.name] + ([f.sub] if f.sub else [])
    return '/' + '/'.join(parts)

def find_objects(D, lib, root, kind, pattern, icase=True, scope='all',
cur_path=(), local_only=False, limit=20000, progress=None,
cancel=None):
    """kind: inst | pin | net | port (ports of hierarchical modules) |
topport. Returns (results, truncated)"""
    rx = re.compile(pattern, re.I if icase else 0); res = []; scanned
= 0; trunc = False
    if kind == 'topport':
        for p in D.modules[root].ports:
            if rx.search(p.name) or rx.search('/' + p.name):
res.append(Found('topport', (), p.name, ''))
        return res, False
    start = () if scope == 'all' else tuple(cur_path); recurse = scope
!= 'module'; pin_cache = {}
    search = rx.search
    for path, m in hier_walk(D, root, start, recurse):
        base = '/' + '/'.join(path) + '/' if path else '/'
        if cancel and cancel(): raise JobCancelled()
        if kind == 'inst':
            for i in m.insts:
                scanned += 1
                if search(i.name if local_only else base + i.name):
res.append(Found('inst', path, i.name, ''))
        elif kind == 'pin':
            for i in m.insts:
                names = pin_cache.get(i.type)
                if names is None:
                    rt = resolve_type(D, lib, i.type); names = [p.name
for p in type_pins(rt, i)] if rt[0] != 'unknown' else [];
pin_cache[i.type] = names
                if not names: names = [c.pin for c in i.conns if c.pin
and c.pin != '*']
                ib = base + i.name + '/'
                for pn in names:
                    scanned += 1
                    if search(pn if local_only else ib + pn):
res.append(Found('pin', path, i.name, pn))
        elif kind == 'net':
            for nm in m.net_order:
                scanned += 1
                if search(nm if local_only else base + nm):
res.append(Found('net', path, nm, ''))
        elif kind == 'port' and path:
            for p in m.ports:
                scanned += 1
                if search(p.name if local_only else base + p.name):
res.append(Found('port', path, p.name, ''))
        if len(res) >= limit: trunc = True; res = res[:limit]; break
        if scanned > 6000000: trunc = True; break
        if progress: progress(min(0.99, scanned / 6000000.0), '%d
matches, %d objects scanned' % (len(res), scanned))
    return res, trunc


# ====================================================================
#  Sample data
# ====================================================================
def sample_lib_text():
    IDX = 'index_1("0.01, 0.05, 0.2"); index_2("0.001, 0.005, 0.02");'
    def tbl(name, v): return '%s(del_3x3){ %s values("%s", "%s",
"%s"); } ' % (name, IDX, v[0], v[1], v[2])
    def nldm(rel):
        return ('timing(){ related_pin : "%s"; timing_sense :
positive_unate; ' % rel +
                tbl('cell_rise', ['0.02, 0.03, 0.06', '0.03, 0.04,
0.07', '0.05, 0.06, 0.09']) + tbl('cell_fall', ['0.02, 0.03, 0.05',
'0.03, 0.04, 0.06', '0.05, 0.06, 0.08']) +
                tbl('rise_transition', ['0.01, 0.02, 0.05', '0.02,
0.03, 0.06', '0.04, 0.05, 0.08']) + tbl('fall_transition', ['0.01,
0.02, 0.04', '0.02, 0.03, 0.05', '0.03, 0.04, 0.07']) + '}')
    def ccs(rel):
        return ('timing(){ related_pin : "%s"; output_current_rise(){
vector(ccs_3x3){ reference_time : 0.05; index_1("0.05");
index_2("0.005"); index_3("0.02, 0.04, 0.08"); values("0.1, 0.4,
0.2"); } } '
                'output_current_fall(){ vector(ccs_3x3){
reference_time : 0.05; index_1("0.05"); index_2("0.005");
index_3("0.02, 0.04, 0.08"); values("-0.1, -0.4, -0.2"); } } '
                'receiver_capacitance1_rise(){ index_1("0.05");
values("0.0021"); } }') % rel
    def ecsm(rel):
        return ('timing(){ related_pin : "%s"; ecsm_waveform_set(){
index_1("0.05"); index_2("0.005"); ecsm_waveform(){ index_1("0.05");
index_2("0.005"); index_3("0.01, 0.02, 0.03"); values("0.1, 0.5,
0.9"); } } '
                'ecsm_capacitance(){ values("0.0021"); } }') % rel
    def timing(model, rel): return nldm(rel) + (ccs(rel) if model ==
'ccs' else ecsm(rel) if model == 'ecsm' else '')
    def setup(rel): return 'timing(){ related_pin : "%s"; timing_type
: setup_rising; rise_constraint(del_3x3){ %s values("0.03, 0.03,
0.04", "0.03, 0.04, 0.04", "0.04, 0.04, 0.05"); } }' % (rel, IDX)
    def cell(name, area, model, pins, groups='', attrs=''):
        ins = [p[0] for p in pins if p[1] == 'input']
        s = '  cell(%s) {\n    area : %s;\n%s%s' % (name, area, attrs, groups)
        for p in pins:
            if len(p) > 4 and p[4] == 'bus':
                s += '    bus(%s) { bus_type : %s; direction : %s; %s
}\n' % (p[0], p[5], p[1], 'capacitance : 0.003;' if p[1] == 'input'
else '')
                continue
            s += '    pin(%s) { direction : %s; %s%s%s' % (p[0], p[1],
'capacitance : 0.002; ' if p[1] == 'input' else 'max_capacitance :
0.1; ', 'function : "%s"; ' % p[2] if p[2] else '', p[3] if len(p) > 3
else '')
            if p[1] == 'output' and not (len(p) > 4 and p[4]):
                for r in ins: s += timing(model, r) + ' '
            elif p[1] == 'input' and len(p) > 4 and p[4] == 'setup': s
+= setup('CK')
            s += '}\n'
        return s + '  }\n'
    o = ('library(demo_tt_0p9v_25c) {\n  delay_model : table_lookup;\n
 time_unit : "1ns"; voltage_unit : "1V"; current_unit : "1mA";
capacitive_load_unit(1,pf); leakage_power_unit : "1nW";\n'
         '  lu_table_template(del_3x3){ variable_1 :
input_net_transition; variable_2 : total_output_net_capacitance; %s
}\n'
         '  output_current_template(ccs_3x3){ variable_1 :
input_net_transition; variable_2 : total_output_net_capacitance;
variable_3 : time; }\n'
         '  type(bus4){ base_type : array; data_type : bit; bit_width
: 4; bit_from : 3; bit_to : 0; downto : true; }\n'
         '  type(bus8){ base_type : array; data_type : bit; bit_width
: 8; bit_from : 7; bit_to : 0; downto : true; }\n') % IDX
    I = lambda n, x='': [n, 'input', '', x]
    O = lambda n, f, x='': [n, 'output', f, x]
    o += cell('INV_X1', 0.53, 'nldm', [I('A'), O('ZN', '!A')])
    o += cell('BUF_X1', 0.80, 'nldm', [I('A'), O('Z', 'A')])
    o += cell('AND2_X1', 1.06, 'ccs', [I('A1'), I('A2'), O('Z', 'A1 & A2')])
    o += cell('AND3_X1', 1.33, 'nldm', [I('A1'), I('A2'), I('A3'),
O('Z', 'A1 & A2 & A3')])
    o += cell('NAND2_X1', 0.80, 'nldm', [I('A1'), I('A2'), O('ZN',
'!(A1 & A2)')])
    o += cell('NAND3_X1', 1.06, 'nldm', [I('A1'), I('A2'), I('A3'),
O('ZN', '!(A1 & A2 & A3)')])
    o += cell('OR2_X1', 1.06, 'nldm', [I('A1'), I('A2'), O('Z', 'A1 | A2')])
    o += cell('NOR2_X1', 0.80, 'nldm', [I('A1'), I('A2'), O('ZN',
'!(A1 | A2)')])
    o += cell('NOR3_X1', 1.06, 'ccs', [I('A1'), I('A2'), I('A3'),
O('ZN', '!(A1 | A2 | A3)')])
    o += cell('XOR2_X1', 1.60, 'ccs', [I('A'), I('B'), O('Z', 'A ^ B')])
    o += cell('XNOR2_X1', 1.60, 'nldm', [I('A'), I('B'), O('ZN', '!(A ^ B)')])
    o += cell('AOI21_X1', 1.06, 'ccs', [I('A1'), I('A2'), I('B'),
O('ZN', '!((A1 & A2) | B)')])
    o += cell('OAI21_X1', 1.06, 'nldm', [I('A1'), I('A2'), I('B'),
O('ZN', '!((A1 | A2) & B)')])
    o += cell('AOI22_X1', 1.33, 'nldm', [I('A1'), I('A2'), I('B1'),
I('B2'), O('ZN', '!((A1 & A2) | (B1 & B2))')])
    o += cell('MUX2_X1', 1.86, 'nldm', [I('I0'), I('I1'), I('S'),
O('Z', '(S & I1) | (!S & I0)')])
    o += cell('MUX4_X1', 4.52, 'ecsm', [I('I0'), I('I1'), I('I2'),
I('I3'), I('S0'), I('S1'), O('Z', '(!S1 & !S0 & I0) | (!S1 & S0 & I1)
| (S1 & !S0 & I2) | (S1 & S0 & I3)')])
    o += cell('HA_X1', 2.66, 'ecsm', [I('A'), I('B'), O('CO', 'A &
B'), O('S', 'A ^ B')])
    o += cell('FA_X1', 4.26, 'ecsm', [I('A'), I('B'), I('CI'), O('CO',
'(A & B) | (CI & (A ^ B))'), O('S', 'A ^ B ^ CI')])
    o += cell('TIEHI_X1', 0.53, 'none', [O('Z', '1')])
    o += cell('TIELO_X1', 0.53, 'none', [O('Z', '0')])
    o += cell('TBUF_X1', 1.60, 'nldm', [I('I'), I('EN'), O('Z', 'I',
'three_state : "!EN"; ')])
    def ff(clk, ns, extra=''): return '    ff(IQ,IQN) { clocked_on :
"%s"; next_state : "%s"; %s}\n' % (clk, ns, extra)
    CKP = ['CK', 'input', '', 'clock : true; ']
    DS = ['D', 'input', '', '', 'setup']
    q = lambda n, f: [n, 'output', f, '', 1]
    o += cell('DFF_X1', 4.52, 'nldm', [DS, CKP, q('Q', 'IQ'), q('QN',
'IQN')], ff('CK', 'D'))
    o += cell('DFFR_X1', 5.32, 'ccs', [DS, CKP, I('RN'), O('Q', 'IQ'),
O('QN', 'IQN')], ff('CK', 'D', 'clear : "!RN"; '))
    o += cell('DFFNR_X1', 5.32, 'nldm', [DS, ['CKN', 'input', '',
'clock : true; '], I('RN'), O('Q', 'IQ')], ff('!CKN', 'D', 'clear :
"!RN"; '))
    o += cell('DFFE_X1', 6.12, 'nldm', [DS, I('EN'), CKP, O('Q',
'IQ')], ff('CK', '(EN & D) | (!EN & IQ)'))
    o += cell('SDFF_X1', 6.40, 'ecsm', [DS, I('SE'), I('SI'), CKP,
O('Q', 'IQ'), O('QN', 'IQN')], ff('CK', '(SE & SI) | (!SE & D)'))
    o += cell('LATQ_X1', 3.20, 'nldm', [I('D'), I('E'), O('Q', 'IQ')],
'    latch(IQ,IQN) { enable : "E"; data_in : "D"; }\n')
    o += cell('ICG_X1', 4.00, 'ccs', [['CK', 'input', '', 'clock :
true; clock_gate_clock_pin : true; '], ['E', 'input', '',
'clock_gate_enable_pin : true; '],
                                      ['TE', 'input', '',
'clock_gate_test_pin : true; '], ['GCLK', 'output', 'CK & IQ',
'clock_gate_out_pin : true; ']],
              '    latch(IQ,IQN) { enable : "!CK"; data_in : "E | TE";
}\n', '    clock_gating_integrated_cell : "latch_posedge";\n')
    o += cell('SRAM_16X8', 120, 'nldm', [['A', 'input', '', '', 'bus',
'bus4'], ['D', 'input', '', '', 'bus', 'bus8'], ['Q', 'output', '',
'', 'bus', 'bus8'], I('CK'), I('WE'), I('CE')],
              '    memory() { type : ram; address_width : 4;
word_width : 8; }\n')
    return o + '}\n'

def sample_netlist_text():
    regs = ''.join('  MUX2_X1  u_m%d (.I0(q[%d]), .I1(d[%d]), .S(en),
.Z(d_hold[%d]));\n  DFFR_X1  u_r%d (.D(d_hold[%d]), .CK(clk),
.RN(rst_n), .Q(q[%d]), .QN());\n' % (i, i, i, i, i, i, i) for i in
range(4))
    return ('// Sample hierarchical gate-level netlist (uses the
built-in demo library).\n'
            '// Two deliberate problems are included so the checks
have something to find:\n'
            '//   * u_dbg reads "dbg_net", which nothing drives, and
leaves ZN empty\n'
            '//   * "ram_q" is driven by the SRAM but never
read\n`timescale 1ns/1ps\n\n'
            'module adder4 (a, b, cin, s, cout);\n  input  [3:0] a,
b;\n  input        cin;\n  output [3:0] s;\n  output       cout;\n
wire c1, c2, c3;\n'
            '  FA_X1 u_fa0 (.A(a[0]), .B(b[0]), .CI(cin), .S(s[0]),
.CO(c1));\n  FA_X1 u_fa1 (.A(a[1]), .B(b[1]), .CI(c1),  .S(s[1]),
.CO(c2));\n'
            '  FA_X1 u_fa2 (.A(a[2]), .B(b[2]), .CI(c2),  .S(s[2]),
.CO(c3));\n  FA_X1 u_fa3 (.A(a[3]), .B(b[3]), .CI(c3),  .S(s[3]),
.CO(cout));\nendmodule\n\n'
            'module reg4 (input clk, input rst_n, input en, input
[3:0] d, output [3:0] q);\n  wire [3:0] d_hold;\n' + regs +
'endmodule\n\n'
            'module flag_logic (input [3:0] v, input sel, output zero,
output odd);\n  wire n0, n1, n2;\n'
            '  NOR2_X1  u_nor0 (.A1(v[0]), .A2(v[1]), .ZN(n0));\n
NOR2_X1  u_nor1 (.A1(v[2]), .A2(v[3]), .ZN(n1));\n  AND2_X1  u_and
(.A1(n0), .A2(n1), .Z(zero));\n'
            '  XOR2_X1  u_x0   (.A(v[0]), .B(v[1]), .Z(n2));\n
AOI21_X1 u_aoi  (.A1(v[2]), .A2(sel), .B(n2),
.ZN(odd));\nendmodule\n\n'
            'module soc_top (input clk, input rst_n, input [3:0] a,
input [3:0] b, input cin, input sel, input en, input oe,\n
   output [3:0] sum, output cout, output zero, output odd, output
[3:0] q, output tri_out);\n'
            '  wire [3:0] s_int;\n  wire [7:0] ram_q;\n  wire gclk,
gnd, dbg_net;\n'
            '  adder4     u_add   (.a(a), .b(b), .cin(cin), .s(s_int),
.cout(cout));\n  TIELO_X1   u_tie   (.Z(gnd));\n'
            '  ICG_X1     u_icg   (.CK(clk), .E(en), .TE(gnd),
.GCLK(gclk));\n  reg4       u_reg   (.clk(gclk), .rst_n(rst_n),
.en(sel), .d(s_int), .q(q));\n'
            '  flag_logic u_flags (.v(q), .sel(sel), .zero(zero), .odd(odd));\n'
            "  SRAM_16X8  u_ram   (.A(a), .D({b, a}), .Q(ram_q),
.CK(clk), .WE(en), .CE(1'b1));\n"
            '  TBUF_X1    u_tb    (.I(odd), .EN(oe), .Z(tri_out));\n
INV_X1     u_dbg   (.A(dbg_net), .ZN());\n  assign sum =
s_int;\nendmodule\n')

# ====================================================================
#  Schematic view: build, layout, wires, painters (Tk canvas + SVG)
# ====================================================================
BIG_NODES = 3000
TILE = 256.0

class Node:
    __slots__ = ('key', 'kind', 'ref', 'sym', 'rt', 'x', 'y', 'pin_sigs', 'idx')
    def __init__(self, key, kind, ref, sym, rt):
        self.key = key; self.kind = kind; self.ref = ref; self.sym =
sym; self.rt = rt; self.x = 0.0; self.y = 0.0; self.pin_sigs = {};
self.idx = 0

class Sig:
    __slots__ = ('id', 'base', 'label', 'stub', 'pins', 'width',
'undriven', 'clock', 'polys', 'bb', 'labels', 'jx', 'idx', 'nd', 'nl')
    def __init__(self, id_, base, label, stub):
        self.id = id_; self.base = base; self.label = label; self.stub
= stub; self.pins = []; self.width = 1; self.undriven = False
        self.clock = False; self.polys = []; self.bb = (0, 0, 0, 0);
self.labels = []; self.jx = []; self.idx = 0; self.nd = 0; self.nl = 0

class View:
    def __init__(self, mod, block):
        self.mod = mod; self.block = block; self.nodes = []; self.sigs
= {}; self.sig_list = []; self.by_key = {}; self.label_mode = None
        self.big = False; self.tiles = None; self.overview = False;
self.special = []

def sig_width(mod, sid):
    if '[' not in sid and sid[:1] != '\\' and sid.isidentifier():
return net_width(mod.nets.get(sid))
    m = re.match(r'^(\\\S+|[A-Za-z_][A-Za-z0-9_$]*)\s*(?:\[\s*(-?\d+)\s*(?::\s*(-?\d+)\s*)?\])?$',
sid)
    if not m: return 1
    if m.group(2) is not None and m.group(3) is not None: return
abs(int(m.group(2)) - int(m.group(3))) + 1
    if m.group(2) is not None: return 1
    return net_width(mod.nets.get(m.group(1)))

def sym_for_inst(D, lib, inst, rt, block):
    k = rt[0]
    if k == 'cell': return cell_symbol(rt[1], block)
    if k == 'module':
        if rt[1]._sym is None: rt[1]._sym = module_symbol(rt[1])
        return rt[1]._sym
    if k == 'prim': return prim_symbol(inst.type, len(inst.conns), block)
    return unknown_symbol(inst)

HUGE_MODULE = int(os.environ.get('NS_PAGE_ABOVE', 900000))   # every
module up to this size is ONE continuous schematic (~2.6 KB of view
memory per cell, so 900k cells stay inside the 3.2 GB budget); only
larger modules fall back to windows
PAGE_SIZE = 2500         # instances per page (only when paging is
switched on with NS_PAGE_ABOVE)
HIER_EAGER_NODES = 30000   # hierarchy tree: levels are expanded down
to this many nodes at load; the rest opens on demand

def n_items(mod): return len(mod.insts) + len(mod.assigns)
def is_paged(mod): return n_items(mod) > HUGE_MODULE
def n_pages(mod): return max(1, -(-n_items(mod) // PAGE_SIZE))
def page_window(mod, page):
    n = n_items(mod); page = max(0, min(page, n_pages(mod) - 1)); a =
page * PAGE_SIZE
    return (a, min(n, a + PAGE_SIZE))

def build_view(D, lib, mod, block=False, window=None):
    v = View(mod, block); v.window = window; ni = len(mod.insts)
    bybase = {}
    def sig(sid, base, label, stub):
        s = v.sigs.get(sid)
        if s is None:
            s = Sig(sid, base, label or sid, stub); s.width = 1 if
stub else sig_width(mod, sid); v.sigs[sid] = s; v.sig_list.append(s)
            if base is not None: bybase.setdefault(base, []).append(s)
        return s
    def add_node(n):
        n.idx = len(v.nodes); v.nodes.append(n); v.by_key[n.key] = n; return n
    def link(node, pin, s, dir_):
        node.pin_sigs.setdefault(pin, []).append(s.id);
s.pins.append((node, pin, dir_))
    def conn(node, pin, expr, dir_):
        if not expr: return
        if expr.isidentifier():
            s = v.sigs.get(expr)
            if s is None: s = sig(expr, expr, expr, False)
            node.pin_sigs.setdefault(pin, []).append(expr);
s.pins.append((node, pin, dir_)); return
        m = SIMPLE_SIG.match(expr)
        if m:
            base = m.group(1); sid = base + (re.sub(r'\s+', '',
m.group(2)) if m.group(2) else ''); s = sig(sid, base, sid, False)
        else:
            s = sig('x:%s.%s' % (node.key, pin), None, expr, True); bb
= bits_of(mod, expr); s.width = 1 if bb.complex else max(1,
len(bb.bits))
        link(node, pin, s, dir_)
    def add_port(p):
        n = mod.nets.get(p.name); nd = add_node(Node('p:' + p.name,
'port', p, port_sym(p.name, p.dir, bool(n and n.vec), n.msb if n else
0, n.lsb if n else 0), None))
        link(nd, 'p', sig(p.name, p.name, p.name, False), 'out' if
p.dir == 'input' else 'in' if p.dir == 'output' else 'io')
    if window is None:
        for p in mod.ports: add_port(p)
        i0, i1, a0, a1 = 0, ni, 0, len(mod.assigns)
    else:
        i0, i1 = min(window[0], ni), min(window[1], ni); a0, a1 =
max(0, window[0] - ni), max(0, window[1] - ni)
    insts = mod.insts
    for k in range(i0, i1):
        inst = insts[k]
        rt = resolve_type(D, lib, inst.type); sym = sym_for_inst(D,
lib, inst, rt, block); pins = type_pins(rt, inst)
        nd = add_node(Node('i:' + (inst.name or '#%d' % k), 'inst',
inst, sym, rt))
        for pin, expr in named_conns(inst, pins):
            sp = sym.pins.get(pin)
            if sp is None: continue
            conn(nd, pin, expr, 'out' if sp['dir'] == 'out' else 'io'
if sp['dir'] == 'io' else 'in')
    for i in range(a0, a1):
        a = mod.assigns[i]
        nd = add_node(Node('a:%d' % i, 'assign', a, assign_sym(), None))
        conn(nd, 'Y', a.lhs, 'out'); conn(nd, 'A', a.rhs, 'in')
    if window is not None:
        for p in mod.ports:
            if p.name in bybase or p.name in v.sigs: add_port(p)
    for nd in v.nodes:
        if nd.kind != 'port': continue
        d = 'out' if nd.ref.dir == 'input' else 'in' if nd.ref.dir ==
'output' else 'io'; have = nd.pin_sigs.get('p', [])
        for s in list(bybase.get(nd.ref.name, ())):
            if s.id != nd.ref.name and s.id not in have: link(nd, 'p', s, d)
    for nd in v.nodes:
        if nd.kind != 'port': continue
        base = v.sigs.get(nd.ref.name)
        if base and len(base.pins) == 1 and len(nd.pin_sigs.get('p', [])) > 1:
            del v.sigs[base.id]; v.sig_list.remove(base);
nd.pin_sigs['p'] = [i for i in nd.pin_sigs['p'] if i != base.id]
    paged = window is not None
    for i, s in enumerate(v.sig_list):
        s.idx = i
        s.nd = sum(1 for p in s.pins if p[2] in ('out', 'io')); s.nl =
sum(1 for p in s.pins if p[2] in ('in', 'io'))
        s.undriven = (not s.stub) and (not paged) and s.nl > 0 and s.nd == 0
        s.clock = any(p[0].sym.pins[p[1]].get('wedge') for p in s.pins)
    v.big = len(v.nodes) > BIG_NODES
    return v

def auto_layout(v):
    N = v.nodes; n = len(N)
    if not n: return
    out = [[] for _ in range(n)]; inn = [[] for _ in range(n)]
    for s in v.sig_list:
        if s.stub: continue
        Dr = [p[0].idx for p in s.pins if p[2] in ('out', 'io')]; Ld =
[p[0].idx for p in s.pins if p[2] in ('in', 'io')]
        if len(Dr) * len(Ld) > 6000: continue
        for d in Dr:
            od = out[d]
            for l in Ld:
                if d != l and not (od and od[-1] == l): od.append(l);
inn[l].append(d)
    color = [0] * n; post = []; back = set()
    def dfs(s0):
        stack = [[s0, 0]]; color[s0] = 1
        while stack:
            fr = stack[-1]; vv = fr[0]
            if fr[1] < len(out[vv]):
                w = out[vv][fr[1]]; fr[1] += 1
                if color[w] == 0: color[w] = 1; stack.append([w, 0])
                elif color[w] == 1: back.add((vv, w))
            else: color[vv] = 2; post.append(vv); stack.pop()
    for nd in N:
        if nd.kind == 'port' and nd.ref.dir == 'input' and
color[nd.idx] == 0: dfs(nd.idx)
    for i in range(n):
        if not inn[i] and color[i] == 0: dfs(i)
    for i in range(n):
        if color[i] == 0: dfs(i)
    layer = [0] * n
    for vv in reversed(post):
        for w in out[vv]:
            if (vv, w) not in back and layer[w] < layer[vv] + 1:
layer[w] = layer[vv] + 1
    max_l = 0
    for nd in N:
        if not (nd.kind == 'port' and nd.ref.dir == 'output'): max_l =
max(max_l, layer[nd.idx])
    for nd in N:
        if nd.kind == 'port' and nd.ref.dir == 'input': layer[nd.idx] = 0
        elif nd.kind == 'port' and nd.ref.dir == 'output':
layer[nd.idx] = max_l + 1
    L = max_l + 2; cols = [[] for _ in range(L)]
    for nd in N: cols[layer[nd.idx]].append(nd.idx)
    pos = [0.0] * n
    for c in cols:
        for k, vv in enumerate(c): pos[vv] = k
    def sweep(fwd):
        rng = range(1, L) if fwd else range(L - 2, -1, -1)
        for l in rng:
            c = cols[l]; nb = inn if fwd else out; key = {}
            for vv in c:
                s = 0; k = 0
                for w in nb[vv]:
                    if layer[w] != layer[vv]: s += pos[w]; k += 1
                key[vv] = s / k if k else pos[vv]
            c.sort(key=lambda a: (key[a], a))
            for k, vv in enumerate(c): pos[vv] = k
    for _ in range(3 if n <= 2000 else 1 if n <= 8000 else 0):
sweep(True); sweep(False)
    gap_x = 80; gap_y = 26; big = n > BIG_NODES
    rows_max = max(60, int(math.sqrt(n) * 1.6)) if big else 10 ** 9
    subcols = []
    for c in cols:
        for i in range(0, max(1, len(c)), rows_max):
subcols.append(c[i:i + rows_max])
    heights = []; widths = []
    for c in subcols:
        heights.append(sum(N[vv].sym.h + gap_y + 14 for vv in c));
widths.append(max([N[vv].sym.w for vv in c] + [0]))
    max_h = max(heights + [1]); x = 0
    for l, c in enumerate(subcols):
        y = (max_h - heights[l]) / 2 + 14 if not big else 14
        for vv in c:
            nd = N[vv]; nd.x = x + (widths[l] - nd.sym.w) / 2; nd.y =
y; y += nd.sym.h + gap_y + 14
        x += widths[l] + gap_x + (0 if big else min(60,
math.ceil(len(c) / 8) * 4))

def apply_layout(v):
    mod = v.mod; lay = mod.layout
    if lay is None: lay = mod.layout = {}
    missing = []
    for nd in v.nodes:
        p = lay.get(nd.key)
        if p: nd.x, nd.y = p
        else: missing.append(nd)
    if len(missing) == len(v.nodes):
        auto_layout(v)
        for nd in v.nodes: lay[nd.key] = (nd.x, nd.y)
        if v.big: index_view(v)
        return
    if missing:
        placed = [nd for nd in v.nodes if nd.key in lay]
        max_x = max(nd.x + nd.sym.w for nd in placed); min_x =
min(nd.x for nd in placed); max_y = max(nd.y + nd.sym.h for nd in
placed)
        grid = {}
        def cells(nd): return [(gx, gy) for gx in range(int((nd.x -
10) // 80), int((nd.x + nd.sym.w + 10) // 80) + 1) for gy in
range(int((nd.y - 24) // 80), int((nd.y + nd.sym.h + 24) // 80) + 1)]
        for nd in placed:
            for c in cells(nd): grid.setdefault(c, []).append(nd)
        def clash(nd):
            for c in cells(nd):
                for o in grid.get(c, ()):
                    if o is not nd and o.x < nd.x + nd.sym.w + 10 and
o.x + o.sym.w + 10 > nd.x and o.y < nd.y + nd.sym.h + 24 and o.y +
o.sym.h + 24 > nd.y: return True
            return False
        for nd in missing:
            sx = sy = 0.0; k = 0
            for sids in nd.pin_sigs.values():
                for sid in sids:
                    sg = v.sigs.get(sid)
                    if sg is None or sg.stub or len(sg.pins) > 40: continue
                    for q in sg.pins:
                        if q[0] is not nd and q[0].key in lay:
                            sx += q[0].x + (q[0].sym.w + 110 if q[2]
== 'out' else -nd.sym.w - 110); sy += q[0].y; k += 1
            if k: nd.x, nd.y = sx / k, sy / k
            else:
                nd.x = max_x + 120 if nd.kind == 'port' and nd.ref.dir
== 'output' else (min_x - 120 - nd.sym.w if nd.kind == 'port' else
min_x); nd.y = max_y + 40
            nd.x = round(nd.x / 10) * 10; nd.y = round(nd.y / 10) *
10; guard = 0
            while guard < 200 and clash(nd): nd.y += 30; guard += 1
            lay[nd.key] = (nd.x, nd.y)
            for c in cells(nd): grid.setdefault(c, []).append(nd)
    if v.big: index_view(v)

def index_view(v):
    t = {}; sp = []
    for nd in v.nodes:
        if nd.kind != 'inst' or (nd.rt and nd.rt[0] == 'module'): sp.append(nd)
        for k in tile_keys(nd.x, nd.y, nd.sym.w, nd.sym.h):
t.setdefault(k, []).append(nd)
    v.tiles = t; v.special = sp

def tile_keys(x, y, w, h, pad=12):
    return [(gx, gy) for gx in range(int((x - pad) // TILE), int((x +
w + pad) // TILE) + 1) for gy in range(int((y - 20) // TILE), int((y +
h + pad + 12) // TILE) + 1)]

def reindex_node(v, nd, ox, oy):
    if v.tiles is None: return
    for k in tile_keys(ox, oy, nd.sym.w, nd.sym.h):
        lst = v.tiles.get(k)
        if lst and nd in lst: lst.remove(nd)
    for k in tile_keys(nd.x, nd.y, nd.sym.w, nd.sym.h):
v.tiles.setdefault(k, []).append(nd)

def nodes_in(v, vis):
    if v.tiles is None: return v.nodes
    gx0 = int(vis[0] // TILE); gx1 = int(vis[2] // TILE); gy0 =
int(vis[1] // TILE); gy1 = int(vis[3] // TILE)
    seen = set(); out = []
    if (gx1 - gx0 + 1) * (gy1 - gy0 + 1) > len(v.tiles):
        it = [(k, l) for k, l in v.tiles.items() if gx0 <= k[0] <= gx1
and gy0 <= k[1] <= gy1]
    else:
        it = [((gx, gy), v.tiles.get((gx, gy))) for gx in range(gx0,
gx1 + 1) for gy in range(gy0, gy1 + 1)]
    for k, lst in it:
        if not lst: continue
        for nd in lst:
            if nd.idx not in seen: seen.add(nd.idx); out.append(nd)
    return out

def ensure_geom(v, s):
    if s.polys is None: sig_geom(s, v.label_mode)


def pin_pos(nd, pin):
    p = nd.sym.pins.get(pin)
    return (nd.x + p['x'], nd.y + p['y'], p['side']) if p else None

def sig_geom(s, labels):
    pts = []
    for nd, pin, d in s.pins:
        q = pin_pos(nd, pin)
        if q: pts.append((q[0], q[1], q[2], d))
    s.polys = []; s.labels = []; s.jx = []
    if not pts: s.bb = (0, 0, 0, 0); return
    x0 = y0 = 1e9; x1 = y1 = -1e9
    if labels or s.stub:
        for x, y, side, d in pts:
            dx = 14 if side == 'R' else -14
            s.polys.append([(x, y), (x + dx, y)])
            s.labels.append((x + dx + (2 if dx > 0 else -2), y - 3,
s.label, 'w' if dx > 0 else 'e'))
            x0 = min(x0, x, x + dx); x1 = max(x1, x, x + dx +
(len(s.label) * 5 if dx > 0 else 0)); y0 = min(y0, y - 10); y1 =
max(y1, y)
            if dx < 0: x0 = min(x0, x + dx - len(s.label) * 5)
        s.bb = (x0, y0, x1, y1); return
    drv = [p for p in pts if p[3] in ('out', 'io')]
    if drv: tx = max(p[0] for p in drv) + 16 + (sum(map(ord, s.id)) % 5) * 4
    else: tx = min(p[0] for p in pts) - 18
    ymin = min(p[1] for p in pts); ymax = max(p[1] for p in pts)
    for x, y, side, d in pts:
        s.polys.append([(x, y), (tx, y)]); x0 = min(x0, x, tx); x1 =
max(x1, x, tx); y0 = min(y0, y); y1 = max(y1, y)
    if ymax > ymin: s.polys.append([(tx, ymin), (tx, ymax)])
    if len(pts) > 2: s.jx = [(tx, p[1]) for p in pts]
    s.bb = (x0, y0, x1, y1)

def layout_sigs(v, label_mode):
    v.label_mode = label_mode
    if v.big:
        for s in v.sig_list: s.polys = None; s.bb = (0, 0, 0, 0)
        return
    for s in v.sig_list: sig_geom(s, label_mode)

def view_bounds(v):
    if not v.nodes: return (0, 0, 200, 100)
    return (min(n.x for n in v.nodes), min(n.y for n in v.nodes) - 16,
max(n.x + n.sym.w for n in v.nodes), max(n.y + n.sym.h for n in
v.nodes) + 16)

THEMES = {
    'light': dict(bg='#fbfcfd', ink='#1a2330', mute='#5f6b7a',
fill='#ffffff', wire='#2f66b3', acc='#0b6f8a', acc_soft='#d9eef4',
bad='#d63030',
                  clock='#d97706', grid='#e8ecf1',
grid_major='#d3dae3', panel='#f3f5f8', ok='#1b7f4b'),
    'dark': dict(bg='#11161d', ink='#dfe6ee', mute='#93a0b1',
fill='#1c2530', wire='#6fa8ee', acc='#52c1dd', acc_soft='#17323b',
bad='#ff6b6b',
                 clock='#fbbf24', grid='#1b222c',
grid_major='#2a3441', panel='#171c23', ok='#5ccf93'),
}

class SvgPainter:
    def __init__(self, T, family='DejaVu Sans'):
        self.T = T; self.out = []; self.family = family
    def line(self, pts, color, width, dash=False, tags=None):
        self.out.append('<polyline points="%s" fill="none" stroke="%s"
stroke-width="%.2f"%s stroke-linejoin="round"/>' % ('
'.join('%.1f,%.1f' % p for p in pts), color, width, '
stroke-dasharray="5 3"' if dash else ''))
    def poly(self, pts, fill, outline, width, dash=False, tags=None):
        self.out.append('<polygon points="%s" fill="%s" stroke="%s"
stroke-width="%.2f"%s stroke-linejoin="round"/>' % ('
'.join('%.1f,%.1f' % p for p in pts), fill or 'none', outline, width,
' stroke-dasharray="5 3"' if dash else ''))
    def oval(self, cx, cy, r, fill, outline, width, tags=None):
        self.out.append('<circle cx="%.1f" cy="%.1f" r="%.1f"
fill="%s" stroke="%s" stroke-width="%.2f"/>' % (cx, cy, r, fill or
'none', outline, width))
    def text(self, x, y, s, size, color, anchor='w', bold=False, tags=None):
        a = {'w': 'start', 'e': 'end', 'c': 'middle'}[anchor]
        self.out.append('<text x="%.1f" y="%.1f" font-size="%.1f"
text-anchor="%s" fill="%s"%s font-family="%s"
dominant-baseline="central">%s</text>' % (x, y, size * 1.3, a, color,
' font-weight="bold"' if bold else '', self.family, esc(s)))

def esc(t): return str(t).replace('&', '&amp;').replace('<',
'&lt;').replace('>', '&gt;').replace('"', '&quot;')

class TkPainter:
    def __init__(self, canvas, vx, vy, z, T, family):
        self.c = canvas; self.vx = vx; self.vy = vy; self.z = z;
self.T = T; self.family = family
    def _f(self, pts):
        out = []
        for x, y in pts: out.append((x - self.vx) * self.z);
out.append((y - self.vy) * self.z)
        return out
    def line(self, pts, color, width, dash=False, tags=None):
        self.c.create_line(*self._f(pts), fill=color, width=max(1,
width * self.z), tags=tags, dash=(4, 3) if dash else None,
joinstyle='round', capstyle='round')
    def poly(self, pts, fill, outline, width, dash=False, tags=None):
        self.c.create_polygon(*self._f(pts), fill=fill or '',
outline=outline, width=max(1, width * self.z), tags=tags, dash=(4, 3)
if dash else None, joinstyle='round')
    def oval(self, cx, cy, r, fill, outline, width, tags=None):
        x = (cx - self.vx) * self.z; y = (cy - self.vy) * self.z; rr =
r * self.z
        self.c.create_oval(x - rr, y - rr, x + rr, y + rr, fill=fill
or '', outline=outline, width=max(1, width * self.z), tags=tags)
    def text(self, x, y, s, size, color, anchor='w', bold=False, tags=None):
        px = int(round(size * 1.3 * self.z))
        if px < 6: return
        self.c.create_text((x - self.vx) * self.z, (y - self.vy) *
self.z, text=s, fill=color, anchor={'w': 'w', 'e': 'e', 'c':
'center'}[anchor],
                           font=(self.family, -px, 'bold' if bold else
'normal'), tags=tags)

def paint_sym(P, sym, ox, oy, T, tags=None, text=True):
    ink = T['ink']; mute = T['mute']
    for pr in sym.prims:
        k = pr[0]
        if k == 'L': P.line([(ox + x, oy + y) for x, y in pr[1]], ink,
1.4, pr[2] if len(pr) > 2 else False, tags)
        elif k == 'P': P.poly([(ox + x, oy + y) for x, y in pr[1]],
T['fill'] if pr[2] else '', ink, 1.5, pr[3], tags)
        elif k == 'O': P.oval(ox + pr[1], oy + pr[2], pr[3], T['fill']
if pr[4] else '', ink, 1.3, tags)
        elif k == 'T' and text: P.text(ox + pr[1], oy + pr[2], pr[3],
pr[4], mute if pr[6] == 'mute' else ink, pr[5], pr[6] == 'bold', tags)

def paint_sig(P, s, st, T, hl, sel_sig, sel_base, text_ok=True, vis=None):
    tags = st.get('tags', False); ovr = st.get('ovr') or {}
    is_sel = s.id == sel_sig; rel = (not is_sel) and sel_base and
s.base == sel_base
    col = T['bad'] if s.undriven else T['clock'] if s.clock else
T['wire']; w = 2.6 if s.width > 1 else 1.4
    oc = ovr.get(s.id) or (ovr.get(s.base) if s.base else None)
    if oc: col = oc; w += 1.6
    elif is_sel or s.id in hl: col = T['acc']; w += 1.0
    elif rel: col = T['acc']
    cull = vis is not None and len(s.polys) > 40
    for poly in s.polys:
        if cull:
            (x1, y1), (x2, y2) = poly[0], poly[-1]
            if max(x1, x2) < vis[0] or min(x1, x2) > vis[2] or max(y1,
y2) < vis[1] or min(y1, y2) > vis[3]: continue
        P.line(poly, col, w, s.undriven, ('s%d' % s.idx, 'wire') if
tags else None)
    for j in (s.jx[:400] if cull else s.jx): P.oval(j[0], j[1], 2.4,
col, col, 1, ('s%d' % s.idx,) if tags else None)
    if text_ok and len(s.labels) < 400:
        for (x, y, t, a) in s.labels: P.text(x, y, t, 8, col, a,
False, ('s%d' % s.idx,) if tags else None)
        if st.get('net_names') and not s.labels and s.polys and not s.stub:
            x, y = s.polys[0][0]; P.text(x + 16, y - 6, s.id + ('
[%d]' % s.width if s.width > 1 else ''), 8, col, 'w', False, ('s%d' %
s.idx,) if tags else None)

def hit_test(v, wx, wy, z):
    """-> ('pin', node, pin) | ('node', node) | ('sig', sig) | None"""
    if v.big and v.overview: return None
    r = max(6.0, 8.0 / z); best = None
    rp = min(r, 12.0)   # pin grab radius in world units: capped so a
zoomed-out gate body is still draggable
    cand = nodes_in(v, (wx - r, wy - r, wx + r, wy + r)) if v.big else v.nodes
    for nd in reversed(cand):
        if nd.x - r <= wx <= nd.x + nd.sym.w + r and nd.y - r <= wy <=
nd.y + nd.sym.h + r:
            for pn, p in nd.sym.pins.items():
                dx = nd.x + p['x'] - wx; dy = nd.y + p['y'] - wy
                if dx * dx + dy * dy <= rp * rp: return ('pin', nd, pn)
            if best is None and nd.x <= wx <= nd.x + nd.sym.w and nd.y
<= wy <= nd.y + nd.sym.h: best = ('node', nd)
    if best: return best
    tol = max(4.0, 6.0 / z)
    if v.big:
        sigs = {}
        for nd in nodes_in(v, (wx - 120, wy - 80, wx + 120, wy + 80)):
            for ids in nd.pin_sigs.values():
                for sid in ids:
                    s = v.sigs.get(sid)
                    if s is not None and sid not in sigs:
ensure_geom(v, s); sigs[sid] = s
        it = list(sigs.values())
    else: it = v.sig_list
    for s in it:
        bb = s.bb
        if not s.polys or wx < bb[0] - tol or wx > bb[2] + tol or wy <
bb[1] - tol or wy > bb[3] + tol: continue
        for poly in s.polys:
            for (x1, y1), (x2, y2) in zip(poly, poly[1:]):
                if x1 == x2 and y1 == y2: continue
                if min(x1, x2) - tol <= wx <= max(x1, x2) + tol and
min(y1, y2) - tol <= wy <= max(y1, y2) + tol:
                    dx = x2 - x1; dy = y2 - y1; t = max(0, min(1, ((wx
- x1) * dx + (wy - y1) * dy) / (dx * dx + dy * dy)))
                    px = x1 + t * dx - wx; py = y1 + t * dy - wy
                    if px * px + py * py <= tol * tol: return ('sig', s)
    return None

def mix_hex(a, b, f):
    try:
        ra, ga, ba = int(a[1:3], 16), int(a[3:5], 16), int(a[5:7],
16); rb, gb, bb = int(b[1:3], 16), int(b[3:5], 16), int(b[5:7], 16)
        return '#%02x%02x%02x' % (int(ra + (rb - ra) * f), int(ga +
(gb - ga) * f), int(ba + (bb - ba) * f))
    except (ValueError, IndexError): return b

def paint_overview(P, v, vis, st, T):
    """density map for modules with too many cells in view"""
    z = st['z']; step = 1
    while TILE * step * z < 6: step *= 2
    agg = {}
    for (gx, gy), lst in v.tiles.items():
        if (gx + 1) * TILE < vis[0] or gx * TILE > vis[2] or (gy + 1)
* TILE < vis[1] or gy * TILE > vis[3]: continue
        k = (gx // step, gy // step); agg[k] = agg.get(k, 0) + len(lst)
    for (kx, ky), c in agg.items():
        f = min(1.0, 0.18 + c / (14.0 * step * step))
        x0 = kx * step * TILE; y0 = ky * step * TILE; x1 = x0 + step *
TILE; y1 = y0 + step * TILE
        P.poly([(x0, y0), (x1, y0), (x1, y1), (x0, y1)],
mix_hex(T['bg'], T['acc'], f), '', 0)
    for nd in v.special:
        if nd.x + nd.sym.w >= vis[0] and nd.x <= vis[2] and nd.y +
nd.sym.h >= vis[1] and nd.y <= vis[3]:
            paint_sym(P, nd.sym, nd.x, nd.y, T, ('n%d' % nd.idx,
'node') if st.get('tags') else None, False)

def paint_view(P, v, vis, st, T):
    """vis=(x0,y0,x1,y1) world rect; st: dict(z, sel_node, sel_sig,
pending, flags, net_names, pin_names, tags, hl, ovr, msel)"""
    z = st['z']; tags = st.get('tags', False); text_ok = z > 0.3
    sel_node = st.get('sel_node'); sel_sig = st.get('sel_sig'); hl =
set(); hlg = st.get('hl') or []; msel = st.get('msel')
    if sel_node and sel_node in v.by_key:
        for ids in v.by_key[sel_node].pin_sigs.values(): hl.update(ids)
    sel_base = v.sigs[sel_sig].base if sel_sig and sel_sig in v.sigs else None
    def hit(bb): return bb[2] >= vis[0] and bb[0] <= vis[2] and bb[3]
>= vis[1] and bb[1] <= vis[3]
    if v.big:
        cand = nodes_in(v, vis)
        if len(cand) > st.get('detail_limit', 1400):
            v.overview = True; paint_overview(P, v, vis, st, T);
paint_marks(P, v, st, T, hlg, msel, vis); return
        v.overview = False; sigs = {}
        for nd in cand:
            for ids in nd.pin_sigs.values():
                for sid in ids:
                    s = v.sigs.get(sid)
                    if s is not None and sid not in sigs:
ensure_geom(v, s); sigs[sid] = s
        wires = [s for s in sigs.values() if s.polys and hit(s.bb)];
nodes_iter = cand
    else:
        v.overview = False; wires = [s for s in v.sig_list if s.polys
and hit(s.bb)]; nodes_iter = v.nodes
    for s in wires: paint_sig(P, s, st, T, hl, sel_sig, sel_base, text_ok, vis)
    for nd in nodes_iter:
        if not (nd.x - 12 <= vis[2] and nd.x + nd.sym.w + 12 >= vis[0]
and nd.y - 18 <= vis[3] and nd.y + nd.sym.h + 20 >= vis[1]): continue
        tg = ('n%d' % nd.idx, 'node') if tags else None
        if nd.key == sel_node: P.poly([(nd.x - 7, nd.y - 7), (nd.x +
nd.sym.w + 7, nd.y - 7), (nd.x + nd.sym.w + 7, nd.y + nd.sym.h + 7),
(nd.x - 7, nd.y + nd.sym.h + 7)], T['acc_soft'], T['acc'], 1.2, True,
('n%d' % nd.idx,) if tags else None)
        paint_sym(P, nd.sym, nd.x, nd.y, T, tg, text_ok)
        flag = st.get('flags', {}).get(nd.key)
        if flag: P.oval(nd.x + nd.sym.w + 3, nd.y - 3, 4.5, T['bad']
if flag == 'error' else T['clock'] if flag == 'warn' else T['mute'],
T['fill'], 1.2, tg)
        if text_ok:
            if nd.kind == 'inst':
                hier = nd.rt and nd.rt[0] == 'module'
                P.text(nd.x, nd.y - 8, ('> ' if hier else '') +
(nd.ref.name or ''), 9, T['bad'] if nd.rt and nd.rt[0] == 'unknown'
else T['ink'], 'w', True, tg)
                if not nd.sym.box or not st.get('block'):
                    lab = nd.ref.type + ('  ·  ' + nd.sym.label if
nd.sym.label and not nd.sym.box and nd.sym.label != nd.ref.type else
'')
                    P.text(nd.x, nd.y + nd.sym.h + 10, lab, 8,
T['mute'], 'w', False, tg)
            if st.get('pin_names') and not nd.sym.box and not nd.sym.port:
                for pn, p in nd.sym.pins.items():
                   P.text(nd.x + (3 if p['side'] == 'L' else p['x'] -
3), nd.y + p['y'] - 6, pn, 7, T['mute'], 'w' if p['side'] == 'L' else
'e', False, tg)
        if z > 0.35:
            pend = st.get('pending')
            for pn, p in nd.sym.pins.items():
                has = bool(nd.pin_sigs.get(pn)); is_p = pend is not
None and pend[0] == nd.key and pend[1] == pn
                col = T['wire'] if has else (T['bad'] if p['dir'] ==
'in' and nd.kind == 'inst' else T['mute'])
                px = nd.x + p['x']; py = nd.y + p['y']
                if is_p: P.oval(px, py, 6.5, T['acc_soft'], T['acc'],
2.0, ('pend',) if tags else None)
                P.oval(px, py, 2.6, col if has else '', col, 1.3, None)
    paint_marks(P, v, st, T, hlg, msel, vis)

def paint_marks(P, v, st, T, hlg, msel, vis):
    """highlight groups and the multi-selection: outlines on nodes,
rings on pins"""
    groups = list(hlg)
    if msel: groups.append(msel)
    for g in groups:
        col = g['color']
        for key in g['nodes']:
            nd = v.by_key.get(key)
            if nd is None or not (nd.x <= vis[2] and nd.x + nd.sym.w
>= vis[0] and nd.y <= vis[3] and nd.y + nd.sym.h >= vis[1]): continue
            if v.overview or st['z'] < 0.08:
                P.oval(nd.x + nd.sym.w / 2, nd.y + nd.sym.h / 2,
max(6, 8 / max(st['z'], 0.001)), col, col, 1)
            else:
                P.poly([(nd.x - 5, nd.y - 5), (nd.x + nd.sym.w + 5,
nd.y - 5), (nd.x + nd.sym.w + 5, nd.y + nd.sym.h + 5), (nd.x - 5, nd.y
+ nd.sym.h + 5)], '', col, 3.0, False)
        for key, pin in g['pins']:
            nd = v.by_key.get(key)
            if nd is None: continue
            q = pin_pos(nd, pin)
            if q: P.oval(q[0], q[1], 7, '', col, 3.0)

# ====================================================================
#  Tk application
# ====================================================================
import gzip
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, font as tkfont

APP_NAME = 'Netlist Studio'
AUTO_CHECK_MAX = 20000
CRASH_LOG = os.path.join(os.path.expanduser('~'), '.netlist_studio_crash.log')
def mem_mb():
    """peak resident memory in MB"""
    try:
        import resource
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss // 1024
    except Exception: return 0
def cur_mb():
    """current resident memory in MB"""
    try:
        with open('/proc/self/statm') as f: return
int(f.read().split()[1]) * (os.sysconf('SC_PAGE_SIZE') // 1024) //
1024
    except Exception: return mem_mb()
def mem_budget_mb():
    """memory this process may grow to before long traversals stop
(85% of RAM / cgroup limit)"""
    tot = None
    try:
        with open('/proc/meminfo') as f: tot =
int(f.readline().split()[1]) // 1024
    except Exception: pass
    try:
        with open('/sys/fs/cgroup/memory.max') as f:
            v = f.read().strip()
            if v.isdigit(): tot = min(tot or 10 ** 9, int(v) // 1048576)
    except Exception: pass
    return int((tot or 8192) * 0.85)
def trim_memory():
    """collect, freeze the long-lived design objects out of the cyclic
GC, and give freed heap back to the OS"""
    try:
        import gc; gc.collect(); gc.freeze()
    except Exception: pass
    try:
        import ctypes; ctypes.CDLL('libc.so.6').malloc_trim(0)
    except Exception: pass
def log_crash(text):
    try:
        with open(CRASH_LOG, 'a') as f: f.write('\n[%s]\n%s\n' %
(time.strftime('%Y-%m-%d %H:%M:%S'), text))
    except OSError: pass


def read_text(path):
    with open(path, 'rb') as f: magic = f.read(2)
    if magic == b'\x1f\x8b':
        with gzip.open(path, 'rb') as f: data = f.read()
        return data.decode('utf-8', 'replace')
    with open(path, 'r', encoding='utf-8', errors='replace',
newline='') as f: return f.read()

def pick_font(cands, default):
    fams = set(tkfont.families())
    for c in cands:
        if c in fams: return c
    return default

def snap_to(v, step=10): return round(v / step) * step

# ====================================================================
#  ECO command layer: library equivalence, Tcl / Python commands
# ====================================================================
VT_RX = re.compile(r'(?<![A-Za-z])(ULVT|ELVT|UHVT|HVT|SVT|RVT|LVT)(?![A-Za-z])',
re.I)
VT_ALIAS = {'ELVT': 'ULVT', 'RVT': 'SVT'}
VT_ORDER = ['ULVT', 'LVT', 'SVT', 'HVT', 'UHVT']          # fastest /
leakiest first


def _unq(s):
    s = (s or '').strip()
    return s[1:-1] if len(s) >= 2 and s[0] == '"' and s[-1] == '"' else s


def cell_vt(cell):
    """threshold-voltage class of a library cell
(ULVT/LVT/SVT/HVT/UHVT) or None"""
    g = _unq(cell.attrs.get('threshold_voltage_group', '')).upper()
    if g:
        g = VT_ALIAS.get(g, g)
        if g in VT_ORDER: return g
    m = VT_RX.search(cell.name)
    if m:
        g = m.group(1).upper(); return VT_ALIAS.get(g, g)
    return None


def cell_family(cell):
    """cell name with the VT token removed: cells of one family differ
only in their VT flavour"""
    return re.sub(r'_+', '_', VT_RX.sub('', cell.name)).strip('_')


def cell_footprint(cell):
    f = _unq(cell.attrs.get('cell_footprint', ''))
    return f or None


def _norm_func(f):
    ast = parse_func(f) if f else None
    if ast is None: return ('s', re.sub(r'\s+', '', f or ''))
    vs = sorted(ast_vars(ast)); return ('t', tuple(vs),
tuple(truth_table(ast, vs)))


def cell_sig(cell):
    """pin names/directions + per-output logic: two cells with the
same signature are drop-in logical equivalents"""
    s = getattr(cell, '_esig', None)
    if s is None:
        pins = tuple(sorted((p.name, p.dir, p.width) for p in cell.pins))
        funcs = tuple(sorted((p.name, _norm_func(p.func), bool(p.tri))
for p in cell.pins if p.dir in ('output', 'inout')))
        seq = (repr(cell.ff and (cell.ff[0],
sorted(cell.ff[1].items()))), repr(cell.latch and (cell.latch[0],
sorted(cell.latch[1].items()))), bool(cell.memory))
        s = cell._esig = (pins, funcs, seq)
    return s


def drive_rank(cell):
    for rx in (r'_X(\d+)(?:P(\d+))?(?!\d)',
r'(?<=[0-9A-Za-z])X(\d+)(?:P(\d+))?(?!\d)', r'(?<=\d)D(\d+)(?!\d)'):
        mm = re.search(rx, VT_RX.sub('', cell.name), re.I)
        if mm: return float(mm.group(1) + ('.' + mm.group(2) if
mm.lastindex and mm.lastindex > 1 and mm.group(2) else ''))
    return 0.0


def equiv_cells(lib, cell):
    """all library cells that are logical drop-in replacements for
`cell` (same pins and logic), weakest first"""
    ix = getattr(lib, '_eqix', None)
    if ix is None:
        ix = lib._eqix = {}
        for n in lib.order:
            c = lib.cells[n]; ix.setdefault(cell_sig(c), []).append(c)
        for k in ix: ix[k].sort(key=lambda c: (c.area, drive_rank(c), c.name))
    return ix.get(cell_sig(cell), [cell])


def footprint_check(a, b):
    """-> (ok, [reasons], derived). Footprint equivalence = same
pin/logic signature and, when the library gives
    cell_footprint, the same footprint name. Libraries without
cell_footprint fall back to the pin/logic signature."""
    why = []
    if cell_sig(a) != cell_sig(b):
        pa = {(p.name, p.dir) for p in a.pins}; pb = {(p.name, p.dir)
for p in b.pins}
        if pa != pb: why.append('pins differ (only in %s: %s; only in
%s: %s)' % (a.name, ', '.join(sorted(x[0] for x in pa - pb)) or '-',
b.name, ', '.join(sorted(x[0] for x in pb - pa)) or '-'))
        else: why.append('logic function / sequential behaviour differs')
    fa, fb = cell_footprint(a), cell_footprint(b)
    if fa or fb:
        if fa != fb: why.append('cell_footprint differs: %s = %s, %s =
%s' % (a.name, fa or '(none)', b.name, fb or '(none)'))
    return (not why), why, not (fa or fb)


def vt_variants(lib, cell):
    """{vt: cell} for the VT flavours of this cell's family that are
logically equivalent"""
    ix = getattr(lib, '_vtix', None)
    if ix is None:
        ix = lib._vtix = {}
        for n in lib.order:
            c = lib.cells[n]; v = cell_vt(c)
            if v: ix.setdefault(cell_family(c), {}).setdefault(v, []).append(c)
    sig = cell_sig(cell); out = {}
    for v, cs in ix.get(cell_family(cell), {}).items():
        for c in cs:
            if cell_sig(c) == sig: out[v] = c; break
    return out


def repeater_info(lib, name):
    """-> (in_pin, out_pin, inverting) when the cell is a single-input
single-output buffer or inverter, else None"""
    c = lib.cells.get(name) if lib else None
    if c is None: return None
    ins = input_pins(c); outs = [p for p in c.pins if p.dir == 'output']
    if len(ins) != 1 or len(outs) != 1 or not outs[0].func or c.ff or
c.latch: return None
    ast = parse_func(outs[0].func)
    if ast is None or ast_vars(ast) != [ins[0].name]: return None
    tt = truth_table(ast, [ins[0].name])
    if tt == [0, 1]: return (ins[0].name, outs[0].name, False)
    if tt == [1, 0]: return (ins[0].name, outs[0].name, True)
    return None


# ---------------- name patterns (asterisk / question-mark globs)
----------------
def has_wild(p): return '*' in p or '?' in p


def glob_rx(p):
    p = p.replace('.*', '*')
    return re.compile(''.join('.*' if ch == '*' else '.' if ch == '?'
else re.escape(ch) for ch in p) + r'\Z', re.S)


def _split_segs(p):
    return [s for s in p.strip().lstrip('/').split('/')]


def new_inst_named(m, type_, name):
    if m.inst(name) is not None or name in m.nets: raise
EcoError('Name "%s" already exists in module %s.' % (name, m.name))
    i = Inst(name, type_); i.prim = type_.lower() in PRIMS;
m.insts.append(i); jpres(m, i, len(m.insts) - 1, True); ix_set(m, i,
True); soft_touch(m, i); return i


def _fresh(m, base, taken=()):
    n = base; k = 1
    while n in m.nets or n in taken or m.inst(n) is not None: n =
'%s_%d' % (base, k); k += 1
    return n


def _simple_expr(e): return re.sub(r'\s+', '', e or '')


# ---------------- usage texts ----------------
USAGE = {
 'get_cells': 'get_cells [-selected] [-of <objects>] [-hier] [-ref
<lib_cell_pattern>] [<inst_pattern | list> ...]\n  Returns
hierarchical instance names. Patterns use * and ?; "a/b*/c" matches
level by level. With -hier a pattern without "/" is matched at every
hierarchy level.\n  -of <objects> (alias -of_objects) returns the
cells of the given pins or nets; with -hier the leaf cells reached
through the hierarchy (the list a previous get_* returned, e.g.
get_cells -of [get_pins -selected]); the patterns then filter the
result.',
 'get_nets': 'get_nets [-selected] [-of <objects>] [-hier]
[<net_pattern | list> ...]\n  Returns hierarchical net names
("u_blk0/w[3]"). A bit select such as w[3] or w[*] is allowed on
vector nets.\n  -of <objects> (alias -of_objects) returns the nets of
the given pins, cells or ports (the list a previous get_* returned,
e.g. get_cells -of [get_pins -selected]); the patterns then filter the
result.',
 'get_pins': 'get_pins [-selected] [-of <objects>] [-hier]
[<hier_pin_pattern | list> ...]\n  Pin patterns look like
"u_blk0/u1*/ZN" or just "A1" (with -hier: at every level).\n  -of
<objects> (alias -of_objects) returns the pins of the given cells or
nets; with -hier the leaf pins reached through the hierarchy (the list
a previous get_* returned, e.g. get_cells -of [get_pins -selected]);
the patterns then filter the result.',
 'get_ports': 'get_ports [-selected] [-of <objects>] [<port_pattern |
list> ...]\n  Returns top-module ports.\n  -selected (all get_*
commands): instead of searching the design, return the objects
currently selected in the schematic or Find results (library cells
selected in the Library tab for get_lib_cells); patterns, if given,
filter that selection.\n  -of <objects> (alias -of_objects) returns
the top ports of the given nets or pins (the list a previous get_*
returned, e.g. get_cells -of [get_pins -selected]); the patterns then
filter the result.',
 'all_fanout': 'all_fanout -from <hier_pin | top_port>
[-trace_through_arch] [-end_pins_only]\n  Load pins on the net driven
by the pin. -trace_through_arch follows combinational timing arcs
transitively and also lists the output pins of the traversed cells;
-end_pins_only returns only the endpoints (sequential/macro inputs,
top outputs, dangling cells).',
 'all_fanin': 'all_fanin -to <hier_pin | top_port>
[-trace_through_arch] [-start_only]\n  Driver pins of the net feeding
the pin. -trace_through_arch follows combinational arcs backwards;
-start_only returns only startpoints (sequential/macro outputs, top
inputs, tie cells).',
 'report_transitive_fanout': 'report_transitive_fanout -from <hier_pin
| top_port> [-levels <n>] [-max_list <n>]\n  Report of the
combinational cone driven by the pin: levels, cell counts per type,
endpoints.',
 'report_transitive_fanin': 'report_transitive_fanin -from <hier_pin |
top_port> [-levels <n>] [-max_list <n>]\n  Report of the combinational
cone feeding the pin (-to is accepted as a synonym of -from).',
 'add_inst': 'add_inst -inst <hier_inst_name | list> -cell
<library_cell_or_module>\n  Adds unconnected instance(s) in the module
reached by the path ("u_blk0/u_new1").',
 'add_port': 'add_port <new_top_port_name | list> [-dir in|out|bi]
[-width <n>]\n  Adds port(s) to the top module. -dir in = input
(default), out = output, bi = inout; -direction is a synonym and
input/output/inout are accepted.',
 'add_net': 'add_net <hier_net_name | list> [-width <n>]\n  Adds
wire(s) ("u_blk0/n_new" is created in the module of u_blk0).',
 'delete_inst': 'delete_inst <hier_inst_pattern | list>\n  Deletes the
matching instances (asterisk patterns allowed).',
 'delete_ports': 'delete_ports <top_port_pattern | list>\n  Deletes
top-module ports and disconnects what they drove.',
 'delete_net': 'delete_net <hier_net_pattern | list>\n  Deletes wires;
connected pins are left open.',
 'eco_add_repeater': 'eco_add_repeater (-net <hier_net> | -from_pin
<hier_driver_pin>) [-to_pins <hier_pins>] -prefix <name> (-buff
<lib_buffer> | -inv_pair <lib_inverter>) -stages <count>
[-auto_nets_create_with_prefix]\n  Inserts a chain of <count> buffers,
or <count> inverter pairs, on the net. By default every load moves
behind the chain; -to_pins moves only those loads (leaf instance pins
on the net at any hierarchy level; deeper sinks get the chain inside
their own module). Instances are <prefix>_buf_<k> / <prefix>_inv_<k>;
with -auto_nets_create_with_prefix the new nets are
<prefix>_net_<k>.',
 'eco_delete_repeater': 'eco_delete_repeater (-from_pin
<hier_driver_pin> | -net <hier_net>) [-stages <n>] [-prefix <name>]
[-force]\n  Removes the chain of buffers / inverters that follows the
pin or net and reconnects its loads. An odd number of inversions is
refused unless -force. -from_pin and -net are mutually exclusive.',
 'flow_load_config': 'flow_load_config <config.mk> [-flow_home
<dir>]\n  Opens an OpenROAD-flow-scripts design config (export VAR =
value lines) as the project. The ORFS flow directory is found
automatically or given with -flow_home.',
 'flow_run': 'flow_run [-stage
<synth|floorplan|place|cts|route|final|all|clean> ...] [-wait]
[-no_load]\n  Runs ORFS stages with make (default: all). -wait blocks
until the flow ended and returns the stage list; -no_load keeps the
results out of the viewer.',
 'flow_stop': 'flow_stop\n  Stops the running flow.',
 'flow_status': 'flow_status\n  Project, tools and which stages have results.',
 'flow_get_var': 'flow_get_var <NAME>\n  Value of a config variable,
expanded (design file, then platform defaults).',
 'flow_set_var': 'flow_set_var <NAME> <value>\n  Overrides a variable
for the following runs (passed to make as NAME=value).',
 'flow_load_netlist': 'flow_load_netlist [-wait]\n  Loads the
synthesized netlist (and the platform Liberty files) into the
viewer.',
 'flow_show_layout': 'flow_show_layout [-stage
<floorplan|place|cts|route|final>] [-wait]\n  Loads a stage result
(DEF, or ODB converted by OpenROAD) into the Layout tab.',
 'all_registers': 'all_registers [-cells] [-clock_pins] [-data_pins]
[-output_pins] [-async_pins] [-edge_triggered] [-level_sensitive]
[-clock <net | port pattern> ...] [<inst_pattern> ...]\n  Sequential
cells of the whole design (found through the Liberty ff / latch
groups). Default: the register cells. -clock_pins / -data_pins /
-output_pins / -async_pins return those pins of every register instead
(data pins include scan and enable inputs). -clock keeps registers
whose clock pin is driven by the given net or top port;
-edge_triggered / -level_sensitive select flops or latches.',
 'all_connected': 'all_connected [-leaf] <nets | pins | ports | list>
...\n  Pins and ports connected to the given nets, or to the nets of
the given pins (the pin itself is included). Default: the pins at the
level of the net, with hierarchical pins of sub-module instances and
the module ports; -leaf follows the net down to the leaf-cell pins and
up/down to the top ports. Example:  all_connected [get_pins -hier
u_*/ZN]',
 'get_selected': 'get_selected [-type <cells|pins|nets|ports> ...]\n
Everything selected in the schematic (or Find results) as hierarchical
names; -type keeps only the given object kinds. The result can feed
the other commands, e.g.  get_nets -of [get_selected -type pins].',
 'clone_inst': 'clone_inst -inst <hier_inst | list> [-num_clones
<count>] [-auto_tie <0|1>]\n  Clones instances next to the original:
inputs share the original nets, outputs are left open. -auto_tie 1
ties inputs that are unconnected on the original to 1\'b0.',
 'select_inst': 'select_inst [-add] [-zoom] <hier_inst_pattern |
list>\n  Selects instances in the GUI (asterisk patterns).',
 'select_pin': 'select_pin [-add] [-zoom] -regexp <hier_pin_pattern>\n
 Selects pins in the GUI.',
 'select_net': 'select_net [-add] [-zoom] <hier_net_pattern | list>\n
Selects nets in the GUI.',
 'deselect': 'deselect -all | <patterns>\n  Clears the GUI selection
(all, or just the matching objects).',
 'size_cell': 'size_cell <hier_inst | list> <library_cell> [-force]\n
Swaps the library cell of the instances. A footprint-incompatible swap
is refused ("non-eq-footprint") unless -force.',
 'eco_size': 'eco_size -inst <hier_inst | list> (-up | -down) [-steps
<n>] [-force]\n  Upsizes / downsizes to the next logically equivalent
cell (by area / drive strength). Checks footprint equivalence.',
 'eco_swap_vt': 'eco_swap_vt -inst <hier_inst | list> -to
<HVT|SVT|LVT|ULVT|faster|slower> [-force]\n  Swaps to another
threshold-voltage flavour of the same cell family (HVT <-> SVT <-> LVT
<-> ULVT). Checks footprint equivalence.',
 'check_netlist_cells': 'check_netlist_cells [-missing_only]\n  Lists
every leaf cell type the loaded netlist instantiates with its instance
count and status: ok, no-footprint (library cell without
cell_footprint) or MISSING (not in the libraries).',
 'get_equiv_cells': 'get_equiv_cells [-selected] -inst <hier_inst> |
-cell <lib_cell>\n  -selected: use the selected instance(s) as
-inst.\n  Lists logically equivalent library cells with their VT
class, area and footprint-equivalence flag.',
 'get_lib_cells': 'get_lib_cells [-selected] [-of <cells>] [<pattern>
...]\n  Library cell names matching the patterns.',
 'set_eco_option': 'set_eco_option [-uniquify yes|no|ask] [-threads N]
[-lib_check strict|warn|off]\n  -lib_check: when a netlist is loaded,
strict (default) aborts if it instantiates leaf cells missing from the
libraries; warn loads it and lists them; off skips the check. -threads
N: worker processes for heavy steps (index builds; 0 = all cores).
What to do when an edit falls inside a module instantiated more than
once: yes = make a private copy (default), no = edit the shared
module, ask = prompt.',
 'source': 'source <file.tcl | file.py>\n  Runs a Tcl or Python command file.',
 'help': 'help [<command>]\n  Lists the commands, or shows usage of one.',
}


tcl_join = tk._join      # proper Tcl list quoting


class CmdError(Exception):
    pass



# ---- multi-core helpers (fork based: workers share the loaded design
copy-on-write, results come back as raw bytes) ----
THREADS = [0]                 # 0 = all cores; set_eco_option -threads N
_FORK_JOB = [None]
def n_workers():
    c = os.cpu_count() or 1
    return max(1, min(c, THREADS[0] or c))
def fork_ok(): return sys.platform != 'win32' and hasattr(os, 'fork')
and n_workers() > 1
def _fork_run(i): return _FORK_JOB[0](i)
def fork_map(fn, n, cancel=None):
    """[fn(0) .. fn(n-1)] computed in parallel worker processes
(serial when forking is unavailable); fn may be a closure.
    With cancel(), the wait is polled so a Cancel kills the workers at once."""
    if n < 2 or not fork_ok(): return [fn(i) for i in range(n)]
    import multiprocessing, warnings
    _FORK_JOB[0] = fn
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            with multiprocessing.get_context('fork').Pool(min(n,
n_workers())) as pool:
                r = pool.map_async(_fork_run, range(n), chunksize=1)
                while True:
                    try: return r.get(0.15)
                    except multiprocessing.TimeoutError:
                        if cancel is not None and cancel(): raise JobCancelled()
    except JobCancelled: raise
    except Exception:
        return [fn(i) for i in range(n)]
    finally: _FORK_JOB[0] = None

COMPACT_MIN = int(os.environ.get('NS_COMPACT_MIN', 150000))   #
modules this large get the compact connectivity index

class CIdx:
    """connectivity index for very large modules: ~16 bytes per
connection bit instead of ~300.
    Entries are sorted (hash<<shift | entry) words; lookups bisect and
verify against the instance, so hash
    collisions are harmless.  Edited instances are patched through an
overlay (see note_dirty)."""
    def __init__(self, shell, m):
        self.sh = shell; self.m = m; self.snap = None; self.packed =
None; self.shift = 0
        self.ii = array('i'); self.ps = array('H'); self.pp =
array('H'); self.pnames = []; self.pid = {}
        self.dirty = {}; self.ov = {}; self.ov_ok = True
        _CIDX_LIVE.add(self)
    def mark(self, insts):
        for i in insts: self.dirty[id(i)] = i
        self.ov_ok = False
    def build(self, entries_of):
        m = self.m; lz = self.lz = m.insts if isinstance(m.insts,
LazyInsts) else None
        if lz is not None:
            snap = array('i', lz.seq); self.snap = None; N = len(snap)
            def slot_entries(slot):
                if slot >= 0 and slot not in lz.cache:
                    out = []
                    for pin, expr in lz.pairs(slot):
                        if expr:
                            for pos, k in enumerate(self.sh.keys_of(m, expr)):
                                if k: out.append((k, pin, pos))
                    return out
                return entries_of(lz.get(slot))
            ent = slot_entries
        else:
            self.snap = snap = list(m.insts); N = len(snap)
            ent = lambda n: entries_of(snap[n])
        def chunk(c, a, b):
            ii = array('i'); ps = array('H'); pp = array('H'); hh =
array('q'); pn = []; pid = {}
            for n in range(a, b):
                sid = snap[n] if lz is not None else n
                for key, pin, pos in ent(sid):
                    p = pid.get(pin)
                    if p is None: p = pid[pin] = len(pn); pn.append(pin)
                    ii.append(sid); ps.append(p); pp.append(pos);
hh.append(hash(key))
            return ii.tobytes(), ps.tobytes(), pp.tobytes(), hh.tobytes(), pn
        # forked workers copy the pages they touch (CPython
refcounts), so the heap roughly doubles: only fork when that fits in
RAM
        k = n_workers() * 3 if (fork_ok() and N >= 300000 and cur_mb()
* 2.1 + 500 < mem_budget_mb()) else 1
        bounds = [(N * c // k, N * (c + 1) // k) for c in range(k)]
        res = fork_map(lambda c: chunk(c, *bounds[c]), k)
        ii = self.ii; ps = self.ps; pp = self.pp; pid = self.pid;
pnames = self.pnames; hs = array('q')
        for (bi, bp, bq, bh, pn) in res:
            a = array('i'); a.frombytes(bi); ii.extend(a)
            b = array('H'); b.frombytes(bp)
            remap = []
            for nm in pn:
                p = pid.get(nm)
                if p is None: p = pid[nm] = len(pnames); pnames.append(nm)
                remap.append(p)
            ps.extend(array('H', [remap[x] for x in b]) if len(pn) > 1
or (remap and remap[0] != 0) else b)
            c = array('H'); c.frombytes(bq); pp.extend(c)
            d = array('q'); d.frombytes(bh); hs.extend(d)
        del res
        n = len(hs); shift = max(1, n.bit_length()); self.shift =
shift; mask = (1 << (62 - shift)) - 1
        L = [((h & mask) << shift) | e for e, h in enumerate(hs)]; del hs
        L.sort(); self.packed = array('q', L); self.mask = mask
    def _verify(self, e, k):
        pin = self.pnames[self.ps[e]]; pos = self.pp[e]; lz = self.lz
        if lz is None:
            inst = self.snap[self.ii[e]]
            if id(inst) in self.dirty: return None
            expr = self.sh.conn_expr_of(inst, pin)
        else:
            v = self.ii[e]
            if v in lz.dead: return None
            inst = lz.cache.get(v) if v >= 0 else lz.extras[-1 - v]
            if inst is not None:
                if id(inst) in self.dirty: return None
                expr = self.sh.conn_expr_of(inst, pin)
            else:
                expr = next((ex for pn, ex in lz.pairs(v) if pn == pin), None)
        ks = self.sh.keys_of(self.m, expr)
        if not (pos < len(ks) and ks[pos] == k): return None
        return (inst if inst is not None else lz.get(v), pin, pos)
    def get(self, k, default=()):
        if not self.ov_ok: self._rebuild_overlay()
        out = []; sh = self.shift; h = hash(k) & self.mask; lo =
bisect.bisect_left(self.packed, h << sh); P = self.packed; n = len(P);
em = (1 << sh) - 1
        while lo < n and (P[lo] >> sh) == h:
            r = self._verify(P[lo] & em, k)
            if r is not None: out.append(r)
            lo += 1
        o = self.ov.get(k)
        if o: out.extend(o)
        return out or default
    def _rebuild_overlay(self):
        ov = {}; m = self.m
        for inst in self.dirty.values():
            if m.inst(inst.name) is not inst: continue
            for key, pin, pos in self.sh.inst_entries(m, inst):
ov.setdefault(key, []).append((inst, pin, pos))
        self.ov = ov; self.ov_ok = True


class EcoShell:
    """Tcl / Python command interface to the ECO engine. Commands take
the same arguments in both languages."""
    def __init__(self, app):
        self.app = app; self.uniq = 'yes'; self.tkc = app.root.tk;
self.out = lambda s, nl=True: None
        self.cmds = {}
        for n in USAGE:
            f = getattr(self, 'c_' + n, None)
            if f: self.cmds[n] = f
        self.pyns = self.py_namespace()

    # ---- plumbing ----
    @property
    def D(self): return self.app.D
    @property
    def lib(self): return self.app.lib
    @property
    def root(self): return self.app.root_mod

    def need_design(self):
        if not self.D: raise CmdError('No netlist is loaded.')

    def flat(self, items):
        out = []
        for it in items:
            if isinstance(it, (list, tuple)): out.extend(self.flat(it))
            elif isinstance(it, str):
                s = it.strip()
                if not s: continue
                if re.search(r'\s', s) or s[0] in '{"':
                    try: out.extend(self.tkc.splitlist(s)); continue
                    except Exception: pass
                out.append(s)
            else: out.append(str(it))
        return out

    def parse(self, name, argv, spec):
        """spec: {'-opt': 'b'|'s'|'l'|'i'}; returns (opts,
positionals) or raises CmdError; '-help' -> usage"""
        argv = list(argv); opts = {}; pos = []; i = 0; names = list(spec)
        while i < len(argv):
            a = argv[i]
            if not isinstance(a, str): pos.append(a); i += 1; continue
            if a in ('-help', '-h', '--help'): opts['help'] = True; i
+= 1; continue
            if a.startswith('-') and len(a) > 1 and not re.match(r'^-\d', a):
                hit = [n for n in names if n == a] or [n for n in
names if n.startswith(a)]
                if len(hit) != 1: raise CmdError('%s: %s option
"%s"\n%s' % (name, 'ambiguous' if hit else 'unknown', a, USAGE[name]))
                o = hit[0]; k = spec[o]
                if k == 'b': opts[o] = True
                else:
                    if i + 1 >= len(argv): raise CmdError('%s: option
%s needs a value\n%s' % (name, o, USAGE[name]))
                    v = argv[i + 1]; i += 1
                    if k == 's': opts[o] = v if isinstance(v, str) else str(v)
                    elif k == 'i':
                        try: opts[o] = int(v)
                        except (TypeError, ValueError): raise
CmdError('%s: option %s needs an integer, got "%s"' % (name, o, v))
                    else: opts.setdefault(o, []).extend(self.flat([v]))
            else: pos.append(a)
            i += 1
        return opts, pos

    def run(self, name, argv):
        """-> result (python value). Raises CmdError / EcoError with a
readable message."""
        f = self.cmds[name]; spec = getattr(self, 's_' + name, {})
        if name.startswith('get_'): spec = dict(spec, **{'-selected': 'b'})
        if name in self.OF_KIND: spec = dict(spec, **{'-of': 'l',
'-of_objects': 'l'})
        opts, pos = self.parse(name, argv, spec)
        if opts.get('help'): return USAGE[name]
        if '-of_objects' in opts: opts['-of'] = opts.get('-of', []) +
opts.pop('-of_objects')
        if '-of' in opts: res = self.get_of(name, opts, pos, f)
        elif opts.get('-selected'): res = self.get_selected(name, opts, pos, f)
        else: res = f(opts, pos)
        k = self.OF_KIND.get(name)
        if k and isinstance(res, list): self.remember(k, res)
        return res

    # ---- typed results: a name alone does not say whether it is a
cell, pin, net or port, so the last result of each get_* is remembered
----
    OF_KIND = {'get_cells': 'cell', 'get_nets': 'net', 'get_pins':
'pin', 'get_ports': 'port', 'get_lib_cells': 'lib'}
    def remember(self, kind, names):
        if not hasattr(self, 'last_kind'): self.last_kind = {}
        self.last_kind[kind] = set(names) if len(names) <= 2000000 else set()

    def classify_obj(self, name, prefer):
        """kinds a name can denote, most likely first: from the
remembered results, else by looking it up in the design"""
        lk = getattr(self, 'last_kind', {}); name = name.lstrip('/')
        known = [k for k in prefer if name in lk.get(k, ())]
        if known: return known
        out = []
        for k in prefer:
            try:
                if k == 'pin':
                    r = self.resolve_pin(name)
                    if r['inst'] is not None or '/' not in name: out.append(k)
                elif k == 'net': self.resolve_net(name); out.append(k)
                elif k == 'cell': self.resolve_inst(name); out.append(k)
                elif k == 'port':
                    if '/' not in name and any(p.name ==
self._split_bit(name)[0] for p in self.D.modules[self.root].ports):
out.append(k)
            except (CmdError, KeyError, StopIteration): pass
        return out

    def get_of(self, name, o, pos, f):
        """get_cells / get_nets / get_pins / get_ports / get_lib_cells
-of <objects>"""
        self.need_design() if name != 'get_lib_cells' else None
        want = self.OF_KIND[name]; hier = '-hier' in o; elems =
self.flat(o['-of']); out = []; seen = set()
        accept = {'cell': ('pin', 'net', 'cell'), 'net': ('pin',
'cell', 'port', 'net'), 'pin': ('cell', 'net', 'pin'), 'port': ('net',
'pin', 'port'), 'lib': ('cell',)}[want]
        def add(n):
            if n not in seen: seen.add(n); out.append(n)
        for e in elems:
            kinds = self.classify_obj(e, accept)
            if not kinds: raise CmdError('%s -of: "%s" is not a pin,
net, cell or port of the design.' % (name, e))
            k = kinds[0]
            if want == 'lib':
                path, m, inst = self.resolve_inst(e.lstrip('/')); add(inst.type)
            elif k == 'pin':
                ref = self.resolve_pin(e.lstrip('/'))
                if want == 'cell':
                    if ref['inst'] is not None:
add(self.hname(ref['path'], ref['inst'].name))
                elif want == 'net':
                    for pth, key in self.pin_nets(ref):
add(self.hname(pth, key))
                elif want == 'port':
                    for pth, key in self.pin_nets(ref):
                        leaves, tops, loc, lp = self.trace(pth, key)
                        for pn, d in tops: add(pn.split('[')[0] if '['
in pn and False else pn)
            elif k == 'net':
                path, m, base, key = self.resolve_net(e.lstrip('/'))
                leaves, tops, loc, lp = self.trace(path, key)
                if want == 'cell':
                    if hier: [add(self.hname(lpth, inst.name)) for
(lpth, lm, inst, pin, pos_, d) in leaves]
                    else: [add(self.hname(path, inst.name)) for (inst,
pin, pos_) in loc]
                elif want == 'pin':
                    if hier: [add(self.pin_full(lpth, inst, pin)) for
(lpth, lm, inst, pin, pos_, d) in leaves]
                    else: [add(self.pin_full(path, inst, pin)) for
(inst, pin, pos_) in loc]
                elif want == 'port':
                    [add(pn) for pn, d in tops]
            elif k == 'cell':
                path, m, inst = self.resolve_inst(e.lstrip('/'))
                if want == 'pin':
                    rt, pins = self.pins_of(inst)
                    for pn in ([x.name for x in pins] or [c.pin for c
in inst.conns if c.pin]): add(self.pin_full(path, inst, pn))
                elif want == 'net':
                    rt, pins = self.pins_of(inst)
                    for pn, expr in named_conns(inst, pins):
                        for kk in (self.keys_of(m, expr) if expr else []):
                            if kk: add(self.hname(path, kk))
                elif want == 'cell': add(self.hname(path, inst.name))
            elif k == 'port':
                if want == 'net':
                    pth, m, base, key = self.resolve_net(e.lstrip('/'));
                    for pk, kk in self.pin_nets({'path': (), 'm':
self.D.modules[self.root], 'inst': None, 'pin': self._split_bit(e)[0],
'bit': self._split_bit(e)[1]}): add(self.hname(pk, kk))
                elif want == 'port': add(e.lstrip('/'))
                elif want == 'pin': add(e.lstrip('/'))
        pats = self.flat(pos)
        if pats:
            rxs = [glob_rx(p.lstrip('/')) for p in pats]
            out = [n for n in out if any(r.match(n) or
r.match(n.rsplit('/', 1)[-1]) for r in rxs)]
        if name == 'get_cells' and '-ref' in o:
            rr = glob_rx(o['-ref']); keep = []
            for n in out:
                try:
                    if rr.match(self.resolve_inst(n)[2].type): keep.append(n)
                except CmdError: pass
            out = keep
        return out

    def c_get_selected(self, o, pos):
        self.need_design(); want = {x.lower().rstrip('s') for x in
o.get('-type', []) + self.flat(pos)}
        by = {'cell': [], 'pin': [], 'net': [], 'port': []}
        for x in self.app.selection_found():
            kind = {'inst': 'cell', 'pin': 'pin', 'net': 'net',
'port': 'port', 'topport': 'port'}.get(x.kind)
            n = found_str(x).lstrip('/')
            if kind and n not in by[kind]: by[kind].append(n)
        for k, v in by.items():
            if v: self.remember(k, v)
        bad = want - set(by) - {''}
        if bad: raise CmdError('get_selected: unknown -type %s (use
cells, pins, nets, ports)' % ', '.join(sorted(bad)))
        return [n for k in ('cell', 'pin', 'net', 'port') if not want
or k in want for n in by[k]]
    s_get_selected = {'-type': 'l'}

    s_all_connected = {'-leaf': 'b'}
    def c_all_connected(self, o, pos):
        self.need_design(); elems = self.flat(pos); out = []; seen =
set(); leaf = '-leaf' in o
        if not elems: raise CmdError('all_connected: give nets, pins
or ports\n' + USAGE['all_connected'])
        def add(n):
            if n not in seen: seen.add(n); out.append(n)
        done = set()
        for e in elems:
            kinds = self.classify_obj(e, ('net', 'pin', 'port'))
            if not kinds: raise CmdError('all_connected: "%s" is not a
net, pin or port of the design.' % e)
            nets = []
            if kinds[0] == 'net':
                path, m, base, key = self.resolve_net(e.lstrip('/'));
keys = [key]
                n = m.nets[base]
                if n.vec and key == base:
                    step = -1 if n.msb >= n.lsb else 1; keys =
['%s[%d]' % (base, i) for i in range(n.msb, n.lsb + step, step)]
                nets = [(path, k) for k in keys]
            elif kinds[0] == 'pin': nets =
self.pin_nets(self.resolve_pin(e.lstrip('/')))
            else:
                b, bit = self._split_bit(e.lstrip('/')); nets =
self.pin_nets({'path': (), 'm': self.D.modules[self.root], 'inst':
None, 'pin': b, 'bit': bit})
            for (pth, key) in nets:
                if (pth, key) in done: continue
                done.add((pth, key))
                leaves, tops, loc, lp = self.trace(pth, key)
                if leaf:
                    for (lpth, lm, inst, pin, pos_, d) in leaves:
add(self.pin_full(lpth, inst, pin))
                    for pn, d in tops: add(pn)
                else:
                    for (inst, pin, pos_) in loc:
add(self.pin_full(pth, inst, pin))
                    for pn, d in lp: add(self.hname(pth, pn))
        self.remember('pin', out)
        return out

    s_all_registers = {'-cells': 'b', '-clock_pins': 'b',
'-data_pins': 'b', '-output_pins': 'b', '-async_pins': 'b',
'-edge_triggered': 'b', '-level_sensitive': 'b', '-clock': 'l'}
    def c_all_registers(self, o, pos):
        self.need_design()
        if not self.lib: raise CmdError('all_registers: load a Liberty
file first; registers are recognised from its ff / latch groups.')
        D = self.D; lib = self.lib; types = {}; alltypes = set()
        for mn in D.order: alltypes |= inst_type_set(D.modules[mn])
        for t in alltypes:
            c = lib.cells.get(t)
            if c is None: continue
            d = classify_cell(c)
            if d.get('kind') != 'seq': continue
            if '-edge_triggered' in o and d['type'] != 'dff': continue
            if '-level_sensitive' in o and d['type'] != 'latch': continue
            types[t] = d
        pats = self.flat(pos); prx = [glob_rx(p.lstrip('/')) for p in pats]
        clk_rx = [glob_rx(p.lstrip('/')) for p in
self.flat(o.get('-clock', []))]
        pin_kinds = [k for k in ('-clock_pins', '-data_pins',
'-output_pins', '-async_pins') if k in o]
        out = []; ccache = {}
        for path, m in hier_walk(D, self.root):
            for inst in insts_of_types(m, set(types)):
                full = self.hname(path, inst.name); d = types[inst.type]
                if prx and not any(r.match(full) or r.match(inst.name)
for r in prx): continue
                if clk_rx:
                    cp = d['clk'][0] if d['clk'] else None; expr =
conn_expr(inst, cp) if cp else None; ok = False
                    for kk in (self.keys_of(m, expr) if expr else []):
                        if not kk: continue
                        names = ccache.get((path, kk))
                        if names is None:
                            names = [self.hname(path, kk)] + [pn for
pn, _ in self.trace(path, kk)[1]]; ccache[(path, kk)] = names
                        if any(r.match(n) for r in clk_rx for n in
names): ok = True; break
                    if not ok: continue
                if not pin_kinds: out.append(full); continue
                for k in pin_kinds:
                    if k == '-clock_pins': pl = [d['clk'][0]] if
d['clk'] else []
                    elif k == '-data_pins': pl = list(d['data']) +
([d['en'][0]] if d['en'] else []) + (list(d['scan']) if d['scan'] else
[])
                    elif k == '-output_pins': pl = [x[0] for x in d['outs']]
                    else: pl = [x[0] for x in (d['clr'], d['pre']) if x]
                    for pn in pl: out.append(full + '/' + pn)
        return out


    SEL_KINDS = {'get_cells': ('inst',), 'get_nets': ('net',),
'get_pins': ('pin',), 'get_ports': ('port', 'topport')}
    def get_selected(self, name, o, pos, f):
        """-selected for the get_* commands: the objects selected in
the schematic / find results, optionally filtered by the patterns"""
        self.need_design() if name != 'get_lib_cells' else None
        pats = self.flat(pos)
        if name == 'get_lib_cells':
            try: sel = [i[2:] for i in self.app.libtree.selection() if
i.startswith('c:')]
            except Exception: sel = []
            names = [n for n in sel if n in (self.lib.cells if
self.lib else {})]
        elif name == 'get_equiv_cells':
            insts = [found_str(x).lstrip('/') for x in
self.app.selection_found() if x.kind == 'inst']
            if not insts: raise CmdError('get_equiv_cells -selected:
no instance is selected.')
            out = []
            for n in insts: out += f({'-inst': n}, [])
            return out
        else:
            kinds = self.SEL_KINDS[name]; names = []
            for x in self.app.selection_found():
                if x.kind in kinds:
                    n = found_str(x).lstrip('/')
                    if n not in names: names.append(n)
        if pats:
            rxs = [glob_rx(p.lstrip('/')) for p in pats]
            names = [n for n in names if any(r.match(n) or
r.match(n.rsplit('/', 1)[-1]) for r in rxs)]
        if name == 'get_cells' and '-ref' in o:
            rr = glob_rx(o['-ref']); keep = []
            for n in names:
                try:
                    if rr.match(self.resolve_inst(n)[2].type): keep.append(n)
                except CmdError: pass
            names = keep
        return names

    # ---- hierarchy navigation ----
    def mod_at(self, path):
        return chain_modules(self.D, self.root, tuple(path))[-1]

    def pins_of(self, inst):
        rt = resolve_type(self.D, self.lib, inst.type); return rt,
type_pins(rt, inst)

    def match_insts(self, pat, hier):
        D = self.D; segs = _split_segs(pat); res = []; rm = D.modules[self.root]
        def pick(m, seg):
            if not has_wild(seg):
                i = m.inst(seg); return [i] if i is not None else []
            rx = glob_rx(seg)
            return m.insts.match(rx) if isinstance(m.insts, LazyInsts)
else [x for x in m.insts if rx.match(x.name)]
        def rec(path, m, k):
            for inst in pick(m, segs[k]):
                if k == len(segs) - 1: res.append((path, inst))
                elif inst.type in D.modules and len(path) < 64:
rec(path + (inst.name,), D.modules[inst.type], k + 1)
        if hier and len(segs) == 1:
            for path, m in hier_walk(D, self.root):
                for inst in pick(m, segs[0]): res.append((path, inst))
        else: rec((), rm, 0)
        return res

    def hname(self, path, name): return '/'.join(list(path) + [name])

    def resolve_inst(self, name):
        """'a/b/u1' -> (path, module_containing, inst)"""
        segs = _split_segs(name); path = []; m = self.D.modules[self.root]
        for s in segs[:-1]:
            i = m.inst(s)
            if i is None or i.type not in self.D.modules: raise
CmdError('Hierarchy "%s" not found (at "%s").' % ('/'.join(segs[:-1]),
s))
            path.append(s); m = self.D.modules[i.type]
        i = m.inst(segs[-1])
        if i is None: raise CmdError('Instance "%s" not found.' % name)
        return tuple(path), m, i

    def resolve_pin(self, name):
        """-> dict(path, m, inst|None, pin, bit|None)"""
        segs = _split_segs(name); D = self.D
        if len(segs) == 1:
            m = D.modules[self.root]; base, bit = self._split_bit(segs[0])
            if not any(p.name == base for p in m.ports): raise
CmdError('"%s" is neither a hierarchical pin nor a top port.' % name)
            return {'path': (), 'm': m, 'inst': None, 'pin': base, 'bit': bit}
        path, m, inst = self.resolve_inst('/'.join(segs[:-1]))
        base, bit = self._split_bit(segs[-1]); rt, pins = self.pins_of(inst)
        if not any(p.name == base for p in pins): raise CmdError('%s
(%s) has no pin "%s".' % ('/'.join(segs[:-1]), inst.type, base))
        return {'path': path, 'm': m, 'inst': inst, 'pin': base, 'bit': bit}

    @staticmethod
    def _split_bit(s):
        mm = re.match(r'^(.*?)\s*\[\s*(-?\d+)\s*\]$', s)
        return (mm.group(1), int(mm.group(2))) if mm else (s, None)

    def resolve_net(self, name):
        segs = _split_segs(name); D = self.D; path = []; m =
D.modules[self.root]
        for s in segs[:-1]:
            i = m.inst(s)
            if i is None or i.type not in D.modules: raise
CmdError('Hierarchy "%s" not found.' % '/'.join(segs[:-1]))
            path.append(s); m = D.modules[i.type]
        base, bit = self._split_bit(segs[-1])
        if base not in m.nets: raise CmdError('Net "%s" not found.' % name)
        n = m.nets[base]
        key = ('%s[%d]' % (base, bit)) if (n.vec and bit is not None) else base
        return tuple(path), m, base, key

    # ---- connectivity ----
    def keys_of(self, m, expr):
        """net-bit keys (msb first) for an expression; None entries
are constants"""
        e = (expr or '').strip()
        if not e: return []
        mm = SIMPLE_SIG.match(e)
        if mm:
            base = mm.group(1); n = m.nets.get(base)
            if mm.group(2):
                ix = re.sub(r'\s+', '', mm.group(2))
                if ':' not in ix: return [base + ix]
            elif n is None or not n.vec: return [base]
        b = bits_of(m, e)
        if b.complex: return []
        return [x[1] if x[0] == 'n' else None for x in b.bits]

    def conn_expr_of(self, inst, pin):
        if inst.positional and not inst.prim:
            rt, pins = self.pins_of(inst)
            for pn, e in named_conns(inst, pins):
                if pn == pin: return e
            return None
        return conn_expr(inst, pin)

    def inst_entries(self, m, inst, cache=None):
        out = []
        rt = resolve_type(self.D, self.lib, inst.type); pins =
type_pins(rt, inst)
        for pin, expr in named_conns(inst, pins):
            if not expr or pin is None: continue
            for pos, k in enumerate(self.keys_of(m, expr)):
                if k: out.append((k, pin, pos))
        return out

    def cix(self, m):
        """per-module connectivity index: key -> [(inst, pin, pos)],
plus alias classes from assigns"""
        c = getattr(m, '_cix', None)
        if c is not None: return c
        D = self.D; lib = self.lib; cache = {}
        compact = len(m.insts) >= COMPACT_MIN or isinstance(m.insts, LazyInsts)
        idx = CIdx(self, m) if compact else {}
        def entries_of(inst):
            t = inst.type; e = cache.get(t)
            if e is None or inst.positional or inst.prim:
                rt = resolve_type(D, lib, t); pins = type_pins(rt,
inst); e = (rt, pins)
                if not (inst.positional or inst.prim): cache[t] = e
            out = []
            for pin, expr in named_conns(inst, e[1]):
                if not expr or pin is None: continue
                for pos, k in enumerate(self.keys_of(m, expr)):
                    if k: out.append((k, pin, pos))
            return out
        if compact: idx.build(entries_of)
        else:
            for inst in m.insts:
                for k, pin, pos in entries_of(inst): idx.setdefault(k,
[]).append((inst, pin, pos))
        parent = {}
        def find(x):
            while parent.get(x, x) != x:
                parent[x] = parent.get(parent[x], parent[x]); x = parent[x]
            return x
        for a in m.assigns:
            lk = self.keys_of(m, a.lhs); rk = self.keys_of(m, a.rhs)
            if len(lk) == len(rk):
                for x, y in zip(lk, rk):
                    if x and y: parent[find(x)] = find(y)
        members = {}
        if parent:
            for k in list(parent) + list(set(parent.values())):
members.setdefault(find(k), set()).add(k)
        c = (idx, find if parent else None, members)
        m._cix = c; return c

    def trace(self, path, key):
        """all leaf pins and top ports on the global net containing (path, key)
        -> (leaves [(path, m, inst, pin, pos, dir)], tops [(portname,
dir)], local [(inst, pin, pos)], local_ports [(name, dir)])"""
        D = self.D; lib = self.lib; leaves = []; tops = []; seen =
set(); local = []; lports = []
        todo = [(tuple(path), key, True)]
        pdc = {}
        def pdefs(inst):
            rt, pins = self.pins_of(inst); return rt, {p.name: p for p in pins}
        while todo:
            p, k, first = todo.pop()
            m = self.mod_at(p); idx, find, members = self.cix(m)
            root_k = find(k) if find else k
            if (p, root_k) in seen: continue
            seen.add((p, root_k)); ks = members.get(root_k, {k}) if
find else [k]
            for kk in ks:
                for inst, pin, pos in idx.get(kk, ()):
                    rt, pd = pdefs(inst); d = pd[pin].dir if pin in pd
else 'input'
                    if first and p == tuple(path): local.append((inst,
pin, pos))
                    if rt[0] == 'module' and len(p) < 64:
                        cm = rt[1]; port = next((q for q in cm.ports
if q.name == pin), None)
                        if port is None: continue
                        n = cm.nets.get(pin); vec = bool(n and n.vec)
                        if vec: ck = '%s[%d]' % (pin, n.msb - pos if
n.msb >= n.lsb else n.msb + pos)
                        else: ck = pin
                        todo.append((p + (inst.name,), ck, False))
                    else: leaves.append((p, m, inst, pin, pos, d))
                base = kk.split('[')[0]
                port = next((q for q in m.ports if q.name == base),
None) if base in m.nets and m.nets[base].dir else None
                if port is not None:
                    if first and p == tuple(path): lports.append((base
if kk == base else kk, port.dir))
                    if p:
                        pm = self.mod_at(p[:-1]); pinst =
pm.inst(p[-1]); rt, pds = self.pins_of(pinst)
                        expr = next((e for pn, e in named_conns(pinst,
pds) if pn == port.name), None)
                        if expr:
                            pk = self.keys_of(pm, expr); n = m.nets.get(base)
                            if n is not None and n.vec and '[' in kk:
                                ix = int(kk[kk.index('[') + 1:-1]);
pos = (n.msb - ix) if n.msb >= n.lsb else (ix - n.msb)
                            else: pos = 0
                            if 0 <= pos < len(pk) and pk[pos]:
todo.append((p[:-1], pk[pos], False))
                    else: tops.append((base if kk == base else kk, port.dir))
        return leaves, tops, local, lports

    def pin_nets(self, ref):
        """(path, key) list for a resolved pin or port"""
        m = ref['m']; path = ref['path']
        if ref['inst'] is None:
            n = m.nets.get(ref['pin'])
            if n is not None and n.vec:
                if ref['bit'] is not None: return [(path, '%s[%d]' %
(ref['pin'], ref['bit']))]
                step = -1 if n.msb >= n.lsb else 1
                return [(path, '%s[%d]' % (ref['pin'], i)) for i in
range(n.msb, n.lsb + step, step)]
            return [(path, ref['pin'])]
        inst = ref['inst']; rt, pins = self.pins_of(inst)
        expr = next((e for pn, e in named_conns(inst, pins) if pn ==
ref['pin']), None)
        ks = [k for k in self.keys_of(m, expr) if k] if expr else []
        if ref['bit'] is not None:
            pd = next(p for p in pins if p.name == ref['pin'])
            pos = (pd.msb - ref['bit']) if pd.msb >= pd.lsb else
(ref['bit'] - pd.msb)
            ks = [ks[pos]] if 0 <= pos < len(ks) else []
        return [(path, k) for k in ks]

    def is_comb(self, rt):
        if rt[0] == 'prim': return True
        if rt[0] == 'cell':
            return classify_cell(rt[1]).get('cat') == 'comb'
        return False

    def pin_full(self, path, inst, pin): return self.hname(path,
inst.name) + '/' + pin

    def cone(self, ref, forward, through, ends_only, max_levels=10 **
9, limit=3000000):
        """traverse; returns dict(levels=[[(name, kind)]], pins=[names
in order], ends=[names], cells={type:count})"""
        D = self.D; seen_nets = set(); order = []; ends = [];
seen_pins = set(); cells = {}; levels = []
        start = self.pin_full(ref['path'], ref['inst'], ref['pin']) if
ref['inst'] is not None else ref['pin']
        seen_pins.add(start)
        frontier = list(self.pin_nets(ref)); lvl = 0
        want = ('input', 'inout') if forward else ('output', 'inout')
        budget = mem_budget_mb(); self.truncated = None; tick = 0
        while frontier and lvl < max_levels and len(order) < limit and
not self.truncated:
            nxt = []; this = []
            for (p, k) in frontier:
                tick += 1
                if tick % 20000 == 0 and cur_mb() > budget:
                    self.truncated = 'memory budget (%d MB) reached
after %d pins' % (budget, len(order)); break
                leaves, tops, _, _ = self.trace(p, k)
                for (lp, lm, inst, pin, pos, d) in leaves:
                    nm = self.pin_full(lp, inst, pin)
                    if nm in seen_pins or d not in want: continue
                    seen_pins.add(nm)
                    rt, pds = self.pins_of(inst); comb = self.is_comb(rt)
                    cont = through and comb
                    outs = []
                    if cont:
                        for pn, expr in named_conns(inst, pds):
                            pd = next((x for x in pds if x.name == pn), None)
                            if pd is None or not expr: continue
                            if (forward and pd.dir in ('output',
'inout') and pn != pin) or ((not forward) and pd.dir in ('input',
'inout') and pn != pin):
                                outs.append((pn, [kk for kk in
self.keys_of(lm, expr) if kk]))
                    endpoint = not (cont and any(o[1] for o in outs))
                    if endpoint: ends.append(nm)
                    if not ends_only or endpoint: order.append(nm)
                    this.append(nm)
                    if cont:
                        cells[inst.type] = cells.get(inst.type, 0) + 1
                        for pn, kl in outs:
                            onm = self.pin_full(lp, inst, pn)
                            if onm not in seen_pins:
                                seen_pins.add(onm)
                                if not ends_only: order.append(onm)
                                this.append(onm)
                            for kk in kl:
                                if (lp, kk) not in seen_nets:
seen_nets.add((lp, kk)); nxt.append((lp, kk))
                for (pn, d) in tops:
                    if (forward and d in ('output', 'inout')) or ((not
forward) and d in ('input', 'inout')):
                        if pn in seen_pins or pn == ref['pin'] and
ref['inst'] is None: continue
                        seen_pins.add(pn); order.append(pn);
ends.append(pn); this.append(pn)
            levels.append(this); frontier = nxt; lvl += 1
            if not through: break
        if not self.truncated and frontier and len(order) >= limit:
self.truncated = 'pin limit (%d) reached' % limit
        if self.truncated: self.out('Warning: traversal stopped early:
%s; the result is partial.' % self.truncated)
        return {'levels': levels, 'pins': order, 'ends': ends, 'cells': cells}

    # ================= query commands =================
    s_get_cells = {'-hier': 'b', '-ref': 's'}
    def c_get_cells(self, o, pos):
        self.need_design(); pats = self.flat(pos) or ['*']; out = [];
seen = set()
        refrx = glob_rx(o['-ref']) if '-ref' in o else None
        for p in pats:
            for path, inst in self.match_insts(p, '-hier' in o):
                if refrx and not refrx.match(inst.type): continue
                n = self.hname(path, inst.name)
                if n not in seen: seen.add(n); out.append(n)
        return out

    def net_names(self, pat, hier):
        D = self.D; segs = _split_segs(pat); out = []
        last = segs[-1]; mm = re.match(r'^(.*?)\[([^\]]*)\]$', last)
        base_pat, idx_pat = (mm.group(1), mm.group(2)) if mm else (last, None)
        brx = glob_rx(base_pat) if has_wild(base_pat) else None
        def in_mod(path, m):
            names = [nm for nm in m.net_order] if brx else ([base_pat]
if base_pat in m.nets else [])
            for nm in names:
                if brx and not brx.match(nm): continue
                n = m.nets.get(nm)
                if n is None: continue
                if idx_pat is None:
                    out.append(self.hname(path, nm))
                elif n.vec:
                    lo, hi = min(n.msb, n.lsb), max(n.msb, n.lsb)
                    if re.match(r'^-?\d+$', idx_pat):
                        if lo <= int(idx_pat) <= hi:
out.append(self.hname(path, '%s[%s]' % (nm, idx_pat)))
                    else:
                        irx = glob_rx(idx_pat)
                        for i in range(hi, lo - 1, -1):
                            if irx.match(str(i)):
out.append(self.hname(path, '%s[%d]' % (nm, i)))
        if len(segs) == 1 and hier:
            for path, m in hier_walk(D, self.root): in_mod(path, m)
        else:
            mods = [((), D.modules[self.root])]
            for s in segs[:-1]:
                nxt = []
                for path, m in mods:
                    rx = glob_rx(s) if has_wild(s) else None
                    for i in child_insts(D, m):
                        if (rx and rx.match(i.name)) or (not rx and
i.name == s): nxt.append((path + (i.name,), D.modules[i.type]))
                mods = nxt
            for path, m in mods: in_mod(path, m)
        return out

    s_get_nets = {'-hier': 'b'}
    def c_get_nets(self, o, pos):
        self.need_design(); out = []; seen = set()
        for p in (self.flat(pos) or ['*']):
            for n in self.net_names(p, '-hier' in o):
                if n not in seen: seen.add(n); out.append(n)
        return out

    s_get_pins = {'-hier': 'b'}
    def c_get_pins(self, o, pos):
        self.need_design(); out = []; seen = set(); cache = {}
        def scan(insts, pin_pat, prx):
            got = []
            for path, inst in insts:
                names = cache.get(inst.type)
                if names is None:
                    rt, pins = self.pins_of(inst); names = [x.name for
x in pins] or [c.pin for c in inst.conns if c.pin]; cache[inst.type] =
names
                for pn in names:
                    if (prx and prx.match(pn)) or (not prx and pn == pin_pat):
                        n = self.pin_full(path, inst, pn)
                        if n not in seen: seen.add(n); got.append(n)
            return got
        for p in (self.flat(pos) or ['*']):
            segs = _split_segs(p); pin_pat = segs[-1]; prx =
glob_rx(pin_pat) if has_wild(pin_pat) else None
            if len(segs) == 1:
                out += scan([(path, i) for path, m in
(hier_walk(self.D, self.root) if '-hier' in o else [((),
self.D.modules[self.root])]) for i in m.insts], pin_pat, prx);
continue
            ip = '/'.join(segs[:-1]); got = scan(self.match_insts(ip,
False), pin_pat, prx)
            if not got and has_wild(ip):      # '*' may also span
hierarchy levels
                drx = re.compile(re.escape(ip).replace(r'\*',
'.*').replace(r'\?', '[^/]') + r'$')
                got = scan([(path, i) for path, m in hier_walk(self.D,
self.root) for i in m.insts if drx.match('/'.join(path + (i.name,)))],
pin_pat, prx)
            out += got
        return out

    def c_get_ports(self, o, pos):
        self.need_design(); out = []
        for p in (self.flat(pos) or ['*']):
            rx = glob_rx(p.lstrip('/'))
            out += [q.name for q in self.D.modules[self.root].ports if
rx.match(q.name) and q.name not in out]
        return out

    def c_get_lib_cells(self, o, pos):
        if not self.lib: raise CmdError('No Liberty file is loaded.')
        out = []
        for p in (self.flat(pos) or ['*']):
            rx = glob_rx(p); out += [n for n in self.lib.order if
rx.match(n) and n not in out]
        return out

    s_all_fanout = {'-from': 's', '-trace_through_arch': 'b',
'-end_pins_only': 'b'}
    def c_all_fanout(self, o, pos):
        self.need_design()
        if '-from' not in o: raise CmdError('all_fanout: -from is
required\n' + USAGE['all_fanout'])
        r = self.cone(self.resolve_pin(o['-from']), True,
'-trace_through_arch' in o, '-end_pins_only' in o)
        return r['pins']

    s_all_fanin = {'-to': 's', '-from': 's', '-trace_through_arch':
'b', '-start_only': 'b'}
    def c_all_fanin(self, o, pos):
        self.need_design(); p = o.get('-to') or o.get('-from')
        if not p: raise CmdError('all_fanin: -to is required\n' +
USAGE['all_fanin'])
        return self.cone(self.resolve_pin(p), False,
'-trace_through_arch' in o, '-start_only' in o)['pins']

    def _report(self, o, forward):
        self.need_design(); p = o.get('-from') or o.get('-to')
        if not p: raise CmdError('-from is required')
        ref = self.resolve_pin(p); r = self.cone(ref, forward, True,
False, o.get('-levels', 10 ** 9)); mx = o.get('-max_list', 40)
        L = ['Transitive %s of %s' % ('fanout' if forward else
'fanin', p), '=' * 60,
             'Levels traversed       : %d' % len(r['levels']), 'Pins
in the cone       : %d' % len(r['pins']),
             'Combinational cells    : %d' % sum(r['cells'].values()),
'%s : %d' % ('Endpoints              ' if forward else 'Startpoints
        ', len(r['ends']))]
        if r['cells']:
            L.append('Cells by type:')
            for t, n in sorted(r['cells'].items(), key=lambda x:
-x[1])[:20]: L.append('  %-24s %d' % (t, n))
        for i, lv in enumerate(r['levels']):
            if i >= 12: L.append('  ... %d more levels' %
(len(r['levels']) - 12)); break
            L.append('Level %d: %d pins' % (i, len(lv)))
        L.append(('Endpoints' if forward else 'Startpoints') + '
(first %d):' % mx)
        for e in r['ends'][:mx]: L.append('  ' + e)
        if len(r['ends']) > mx: L.append('  ... %d more' %
(len(r['ends']) - mx))
        return '\n'.join(L)
    s_report_transitive_fanout = {'-from': 's', '-levels': 'i',
'-max_list': 'i'}
    def c_report_transitive_fanout(self, o, pos): return self._report(o, True)
    s_report_transitive_fanin = {'-from': 's', '-to': 's', '-levels':
'i', '-max_list': 'i'}
    def c_report_transitive_fanin(self, o, pos): return self._report(o, False)

    # ================= editing helpers =================
    def ask_u(self):
        if self.uniq == 'ask': return self.app.ask_uniquify
        if self.uniq == 'no': return lambda n, c: 'all'
        return lambda n, c: 'uniquify'

    def do_edit(self, label, path_sets, fn):
        """run fn(D) as one undoable edit. path_sets: list of paths
whose modules (and parents) are touched"""
        D = self.D; names = set()
        for p in path_sets: names.update(affected_modules(D,
self.root, HPin(tuple(p), None, '')))
        res = self.app.edit(label, sorted(names), fn, raise_errors=True)
        return res

    def modules_for(self, path, log, created):
        return _walk(self.D, self.root, tuple(path), 0, self.ask_u(),
log, created)

    def split_new(self, name):
        segs = _split_segs(name)
        return tuple(segs[:-1]), segs[-1]

    # ================= editing commands =================
    s_add_inst = {'-inst': 'l', '-cell': 's'}
    def c_add_inst(self, o, pos):
        self.need_design(); names = o.get('-inst', []) + self.flat(pos)
        if not names or '-cell' not in o: raise CmdError('add_inst:
-inst and -cell are required\n' + USAGE['add_inst'])
        cell = o['-cell']
        if resolve_type(self.D, self.lib, cell)[0] == 'unknown': raise
CmdError('add_inst: "%s" is neither a library cell nor a module.' %
cell)
        items = [self.split_new(n) for n in names]
        def fn():
            log = []; created = []; out = []
            for path, nm in items:
                m = self.modules_for(path, log, created)[-1]
                new_inst_named(m, cell, nm);
out.append(self.hname(path, nm)); log.append('ECO: add_inst %s (%s)' %
(out[-1], cell))
            self.D.eco_log.extend(log); return out, created
        res = self.do_edit('add_inst', [p for p, _ in items], fn)
        return res[0] if isinstance(res, tuple) else []

    s_add_port = {'-direction': 's', '-dir': 's', '-width': 'i'}
    def c_add_port(self, o, pos):
        self.need_design(); names = self.flat(pos)
        if not names: raise CmdError('add_port: give the new port
name(s)\n' + USAGE['add_port'])
        d = {'in': 'input', 'input': 'input', 'out': 'output',
'output': 'output', 'bi': 'inout', 'inout':
'inout'}.get((o.get('-dir') or o.get('-direction') or
'input').lower())
        if not d: raise CmdError('add_port: -dir must be in, out or bi
(input, output, inout also accepted)')
        w = o.get('-width', 1)
        def fn():
            m = self.D.modules[self.root]
            for nm in names:
                if nm in m.nets or m.inst(nm): raise EcoError('Name
"%s" already exists in %s.' % (nm, m.name))
                add_port(self.D, m, nm, d, (w - 1, 0) if w > 1 else
None); self.D.eco_log.append('ECO: add_port %s (%s)' % (nm, d))
            return names
        self.do_edit('add_port', [()], fn); return names

    s_add_net = {'-width': 'i'}
    def c_add_net(self, o, pos):
        self.need_design(); names = self.flat(pos)
        if not names: raise CmdError('add_net: give the net name(s)\n'
+ USAGE['add_net'])
        w = o.get('-width', 1); items = [self.split_new(n) for n in names]
        def fn():
            log = []; created = []
            for path, nm in items:
                m = self.modules_for(path, log, created)[-1]
                if nm in m.nets or m.inst(nm): raise EcoError('Name
"%s" already exists in %s.' % (nm, m.name))
                add_net(m, nm, (w - 1, 0) if w > 1 else None);
log.append('ECO: add_net %s' % self.hname(path, nm))
            self.D.eco_log.extend(log); return None, created
        self.do_edit('add_net', [p for p, _ in items], fn); return
[self.hname(p, n) for p, n in items]

    def c_delete_inst(self, o, pos):
        self.need_design(); pats = self.flat(pos)
        if not pats: raise CmdError('delete_inst: give instance
name(s)\n' + USAGE['delete_inst'])
        found = []; seen = set()
        for p in pats:
            r = self.match_insts(p, False)
            if not r and not has_wild(p): raise CmdError('delete_inst:
instance "%s" not found.' % p)
            for path, inst in r:
                k = (path, inst.name)
                if k not in seen: seen.add(k); found.append(k)
        if not found: return []
        def fn():
            log = []; created = []
            for path, nm in found:
                m = self.modules_for(path, log, created)[-1];
remove_instance(m, nm)
                if m.layout: m.layout.pop('i:' + nm, None)
                log.append('ECO: delete_inst %s' % self.hname(path, nm))
            self.D.eco_log.extend(log); return None, created
        self.do_edit('delete_inst', [p for p, _ in found], fn)
        return [self.hname(p, n) for p, n in found]

    def c_delete_ports(self, o, pos):
        self.need_design(); pats = self.flat(pos); m =
self.D.modules[self.root]; out = []
        for p in pats:
            rx = glob_rx(p.lstrip('/')); hits = [q.name for q in
m.ports if rx.match(q.name)]
            if not hits and not has_wild(p): raise
CmdError('delete_ports: port "%s" not found.' % p)
            out += [h for h in hits if h not in out]
        if not out: return []
        def fn():
            for nm in out: remove_port(self.D, m, nm);
self.D.eco_log.append('ECO: delete_ports %s' % nm)
            return out
        self.do_edit('delete_ports', [()], fn); return out

    def c_delete_net(self, o, pos):
        self.need_design(); pats = self.flat(pos); names = []
        for p in pats:
            r = self.net_names(p, False)
            if not r and not has_wild(p): raise CmdError('delete_net:
net "%s" not found.' % p)
            names += [n for n in r if n not in names]
        items = [self.split_new(n) for n in names]
        if not items: return []
        def fn():
            log = []; created = []
            for path, nm in items:
                m = self.modules_for(path, log, created)[-1]; base =
self._split_bit(nm)[0]
                n = m.nets.get(base)
                if n is None: continue
                if n.dir: raise EcoError('"%s" is a port of %s; use
delete_ports.' % (nm, m.name))
                remove_net(m, base); log.append('ECO: delete_net %s' %
self.hname(path, base))
            self.D.eco_log.extend(log); return None, created
        self.do_edit('delete_net', [p for p, _ in items], fn); return names

    # ---- swapping / sizing ----
    def swap_targets(self, patterns):
        out = []; seen = set()
        for p in patterns:
            r = self.match_insts(p, True if '/' not in p else False)
            if not r: raise CmdError('Instance "%s" not found.' % p)
            for path, inst in r:
                if (path, inst.name) not in seen: seen.add((path,
inst.name)); out.append((path, inst))
        return out

    def swap_plan(self, targets, chooser, force, on_noneq=None):
        """chooser(cell)->new cell name or None. Returns [(path, inst,
old, new)] after the footprint check"""
        lib = self.lib
        if not lib: raise CmdError('A Liberty file is needed for cell swaps.')
        plan = []; bad = []
        for path, inst in targets:
            old = lib.cells.get(inst.type)
            if old is None: raise CmdError('%s is not a library cell
(%s).' % (self.hname(path, inst.name), inst.type))
            nn = chooser(old)
            if nn is None: continue
            new = lib.cells.get(nn)
            if new is None: raise CmdError('Library cell "%s" not found.' % nn)
            if new is old: continue
            ok, why, derived = footprint_check(old, new)
            if not ok: bad.append((self.hname(path, inst.name),
old.name, new.name, why))
            plan.append((path, inst, old, new, ok))
        if bad:
            msg = 'non-eq-footprint: %d swap(s) are not
footprint-equivalent:\n' % len(bad) + '\n'.join('  %s: %s -> %s\n
%s' % (n, a, b, '; '.join(w)) for n, a, b, w in bad[:12])
            if len(bad) > 12: msg += '\n  ... %d more' % (len(bad) - 12)
            if not force and not (on_noneq and on_noneq(msg)): raise
CmdError(msg + '\nNothing was changed. Use -force to swap anyway.')
        return plan

    def apply_swaps(self, label, plan):
        if not plan: return []
        def fn():
            log = []; created = []; out = []
            for path, inst, old, new, ok in plan:
                m = self.modules_for(path, log, created)[-1]; i2 =
m.inst(inst.name)
                jinst(i2); i2.type = new.name; soft_touch(m, i2)
                dead = [c.pin for c in i2.conns if c.pin and c.pin not
in new.pin_map]
                if dead: raise EcoError('%s: connected pin(s) %s do
not exist on %s.' % (self.hname(path, inst.name), ', '.join(dead),
new.name))
                out.append(self.hname(path, inst.name));
log.append('ECO: %s %s: %s -> %s%s' % (label, out[-1], old.name,
new.name, '' if ok else '  [non-eq-footprint, forced]'))
            self.D.eco_log.extend(log); return out, created
        res = self.do_edit(label, [p for p, _, _, _, _ in plan], fn)
        return res[0] if isinstance(res, tuple) else []

    s_size_cell = {'-force': 'b'}
    def c_size_cell(self, o, pos):
        self.need_design(); a = self.flat(pos)
        if len(a) < 2: raise CmdError('size_cell: give instance(s) and
a library cell\n' + USAGE['size_cell'])
        cell = a[-1]; tg = self.swap_targets(a[:-1])
        plan = self.swap_plan(tg, lambda old: cell, '-force' in o,
getattr(self.app, 'confirm_noneq', None))
        return self.apply_swaps('size_cell', plan)

    def size_chooser(self, up, steps):
        def ch(old):
            lst = [c for c in equiv_cells(self.lib, old) if cell_vt(c)
== cell_vt(old)]
            if old not in lst: lst.append(old); lst.sort(key=lambda c:
(c.area, drive_rank(c), c.name))
            i = lst.index(old); seq = lst[i + 1:] if up else lst[:i][::-1]
            if not seq: raise CmdError('%s is already the %s
equivalent cell (%s).' % (old.name, 'largest' if up else 'smallest',
', '.join(c.name for c in lst)))
            pick = None
            for n in range(steps):                 # prefer
footprint-equivalent cells; fall back to the plain next size (then
checked / confirmed)
                nxt = next((c for c in seq if footprint_check(old if
pick is None else pick, c)[0]), None) or seq[0]
                pick = nxt; seq = seq[seq.index(nxt) + 1:] if n + 1 <
steps else seq
                if not seq and n + 1 < steps: raise CmdError('%s: only
%d size step(s) available.' % (old.name, n + 1))
            return pick.name
        return ch

    s_eco_size = {'-inst': 'l', '-up': 'b', '-down': 'b', '-steps':
'i', '-force': 'b'}
    def c_eco_size(self, o, pos):
        self.need_design(); pats = o.get('-inst', []) + self.flat(pos)
        if not pats or ('-up' in o) == ('-down' in o): raise
CmdError('eco_size: give -inst and exactly one of -up / -down\n' +
USAGE['eco_size'])
        tg = self.swap_targets(pats)
        plan = self.swap_plan(tg, self.size_chooser('-up' in o,
o.get('-steps', 1)), '-force' in o, getattr(self.app, 'confirm_noneq',
None))
        return self.apply_swaps('eco_size', plan)

    def vt_chooser(self, to):
        want = to.upper(); want = VT_ALIAS.get(want, want)
        def ch(old):
            cv = cell_vt(old)
            if cv is None: raise CmdError('%s has no recognisable VT
class (name token HVT/SVT/LVT/ULVT or threshold_voltage_group).' %
old.name)
            var = vt_variants(self.lib, old)
            if want in ('FASTER', 'SLOWER'):
                i = VT_ORDER.index(cv) + (-1 if want == 'FASTER' else 1)
                cands = [v for v in var if (VT_ORDER.index(v) <
VT_ORDER.index(cv)) == (want == 'FASTER') and v != cv]
                if not cands: raise CmdError('%s: no %s VT flavour in
the library.' % (old.name, want.lower()))
                tgt = min(cands, key=lambda v: abs(VT_ORDER.index(v) -
VT_ORDER.index(cv)))
            else:
                if want not in VT_ORDER: raise CmdError('Unknown VT
class "%s" (use HVT, SVT, LVT, ULVT, faster, slower).' % to)
                tgt = want
            if tgt == cv: return None
            if tgt not in var: raise CmdError('%s: the library has no
%s flavour (available: %s).' % (old.name, tgt, ', '.join(sorted(var,
key=VT_ORDER.index))))
            return var[tgt].name
        return ch

    s_eco_swap_vt = {'-inst': 'l', '-to': 's', '-force': 'b'}
    def c_eco_swap_vt(self, o, pos):
        self.need_design(); pats = o.get('-inst', []) + self.flat(pos)
        if not pats or '-to' not in o: raise CmdError('eco_swap_vt:
-inst and -to are required\n' + USAGE['eco_swap_vt'])
        tg = self.swap_targets(pats)
        plan = self.swap_plan(tg, self.vt_chooser(o['-to']), '-force'
in o, getattr(self.app, 'confirm_noneq', None))
        return self.apply_swaps('eco_swap_vt', plan)

    s_get_equiv_cells = {'-inst': 's', '-cell': 's'}
    def c_get_equiv_cells(self, o, pos):
        if not self.lib: raise CmdError('No Liberty file is loaded.')
        if '-inst' in o: _, _, inst = self.resolve_inst(o['-inst']);
name = inst.type
        elif '-cell' in o: name = o['-cell']
        else: raise CmdError('get_equiv_cells: -inst or -cell
required\n' + USAGE['get_equiv_cells'])
        c = self.lib.cells.get(name)
        if c is None: raise CmdError('"%s" is not a library cell.' % name)
        out = []
        for e in equiv_cells(self.lib, c):
            ok, why, der = footprint_check(c, e)
            out.append('%s  vt=%s  area=%s  %s' % (e.name, cell_vt(e)
or '-', e.area, 'footprint-eq' if ok else 'NON-EQ-FOOTPRINT'))
        return out

    # ---- repeaters ----
    def net_loads(self, m, expr_key, path):
        """(driver, loads, complex_loads, port_info) for the simple
net/bit `expr_key` inside m"""
        drv = None; loads = []; cplx = []
        base = expr_key.split('[')[0]
        if len(m.insts) >= 2000:            # connectivity index
instead of scanning every instance
            idx = self.cix(m)[0]; pdc = {}
            for inst, pin, _pos in idx.get(expr_key, ()):
                rt, pins = self.pins_of(inst); pd = pdc.get(inst.type)
                if pd is None or inst.positional: pd = {p.name: p for
p in pins}; pdc[inst.type] = pd
                if pin not in pd: continue
                expr = self.conn_expr_of(inst, pin)
                if expr and _simple_expr(expr) == expr_key:
                    if pd[pin].dir == 'output': drv = (inst, pin)
                    else: loads.append((inst, pin))
                elif expr: cplx.append((inst, pin))
            port = next((p for p in m.ports if p.name == base), None)
            return drv, loads, cplx, port
        for inst in m.insts:
            rt, pins = self.pins_of(inst); pd = {p.name: p for p in pins}
            for pin, expr in named_conns(inst, pins):
                if not expr or pin not in pd or base not in expr: continue
                if _simple_expr(expr) == expr_key:
                    d = pd[pin].dir
                    if d == 'output': drv = (inst, pin)
                    else: loads.append((inst, pin))
                elif expr_key in self.keys_of(m, expr): cplx.append((inst, pin))
        port = next((p for p in m.ports if p.name == base), None)
        return drv, loads, cplx, port

    s_eco_add_repeater = {'-net': 's', '-from_pin': 's', '-to_pins':
'l', '-prefix': 's', '-buff': 's', '-inv_pair': 's', '-stages': 'i',
'-auto_nets_create_with_prefix': 'b'}
    def c_eco_add_repeater(self, o, pos):
        self.need_design()
        if ('-net' in o) == ('-from_pin' in o): raise
CmdError('eco_add_repeater: give exactly one of -net / -from_pin\n' +
USAGE['eco_add_repeater'])
        if ('-buff' in o) == ('-inv_pair' in o): raise
CmdError('eco_add_repeater: give exactly one of -buff / -inv_pair\n' +
USAGE['eco_add_repeater'])
        if '-prefix' not in o or '-stages' not in o or o['-stages'] <
1: raise CmdError('eco_add_repeater: -prefix and -stages (>= 1) are
required\n' + USAGE['eco_add_repeater'])
        if '-net' in o:
            path, m, base, key = self.resolve_net(o['-net']); n = m.nets[base]
            if n.vec and key == base: raise
CmdError('eco_add_repeater: "%s" is a bus; name one bit, e.g. %s[3].'
% (o['-net'], o['-net']))
            drv_pin = None
        else:
            ref = self.resolve_pin(o['-from_pin'])
            if ref['inst'] is None: raise CmdError('eco_add_repeater:
-from_pin must be an instance pin (use -net for a top port net).')
            ks = self.pin_nets(ref)
            if len(ks) != 1: raise CmdError('eco_add_repeater: pin %s
is not connected to exactly one simple net bit.' % o['-from_pin'])
            path, key = ks[0]; m = ref['m']; drv_pin =
(ref['inst'].name, ref['pin'])
        cell = o.get('-buff') or o['-inv_pair']; info =
repeater_info(self.lib, cell)
        if info is None: raise CmdError('eco_add_repeater: "%s" is not
a single-input buffer/inverter library cell.' % cell)
        inv = info[2]
        if '-buff' in o and inv: raise CmdError('eco_add_repeater:
-buff needs a non-inverting buffer; %s is an inverter (use
-inv_pair).' % cell)
        if '-inv_pair' in o and not inv: raise
CmdError('eco_add_repeater: -inv_pair needs an inverter; %s is a
buffer (use -buff).' % cell)
        to = [self.resolve_pin(x) for x in o.get('-to_pins', [])]
        for t in to:
            if t['inst'] is None: raise CmdError('eco_add_repeater:
-to_pins must be instance pins (%s is a port; to buffer a port, insert
on its net without -to_pins).' % t['pin'])
        far = [t for t in to if t['path'] != path]
        if far:                                   # sink pins at other
hierarchy levels: insert the chain at the sink's own level (on the
local net that carries this global net)
            leaves = {(tuple(l[0]), l[2].name, l[3]) for l in
self.trace(path, key)[0]}
            groups = {}
            for t in to:
                if t['path'] == path: groups.setdefault((path, key),
[]).append(t); continue
                if (tuple(t['path']), t['inst'].name, t['pin']) not in leaves:
                    raise CmdError('eco_add_repeater: pin %s is not a
load of net /%s (it must be a leaf pin on that net, at any hierarchy
level).' % ('/'.join(tuple(t['path']) + (t['inst'].name, t['pin'])),
'/'.join(tuple(path) + (key,))))
                ks = self.pin_nets(t)
                if len(ks) != 1: raise CmdError('eco_add_repeater: pin
%s is not connected to exactly one simple net bit.' % t['pin'])
                groups.setdefault((tuple(ks[0][0]), ks[0][1]), []).append(t)
            res = []
            for gi, ((gp, gk), ts) in enumerate(groups.items()):
                res += self.add_repeater(gp, gk, drv_pin if gp == path
else None, [(t['inst'].name, t['pin']) for t in ts], o['-prefix'] +
('_h%d' % (gi + 1) if len(groups) > 1 else ''), cell, info,
o['-stages'] * (2 if inv else 1), '-auto_nets_create_with_prefix' in
o, 'inv' if inv else 'buf')
            return res
        return self.add_repeater(path, key, drv_pin, [(t['inst'].name,
t['pin']) for t in to], o['-prefix'], cell, info, o['-stages'] * (2 if
inv else 1), '-auto_nets_create_with_prefix' in o, 'inv' if inv else
'buf')

    def add_repeater(self, path, key, drv_pin, to_pins, prefix, cell,
info, count, auto_nets, tag):
        D = self.D; ipin, opin, _ = info
        def fn():
            log = []; created = []
            m = self.modules_for(path, log, created)[-1]
            drv, loads, cplx, port = self.net_loads(m, key, path)
            if drv_pin: drv = next(((i, p) for i in
[m.inst(drv_pin[0])] for p in [drv_pin[1]] if i is not None), drv)
            cplx_d = []; cplx_l = []                    # complex
connections: output pins (drivers on a bus bit) vs. load pins
            for ci, cp in cplx:
                rt_, pins_ = self.pins_of(ci); pd_ = next((x for x in
pins_ if x.name == cp), None)
                (cplx_d if pd_ is not None and pd_.dir == 'output'
else cplx_l).append((ci, cp))
            cplx = cplx_l; cplx_set = {(id(i), p) for i, p in cplx}
            def sub_ok(e): return e is not None and subst_bit(m, e,
key, 'X') is not None
            drv_c = cplx_d[0] if (drv is None and len(cplx_d) == 1 and
sub_ok(conn_expr(cplx_d[0][0], cplx_d[0][1]))) else None
            drv_a = None
            if drv is None and drv_c is None and not cplx_d:
                da = [a for a in m.assigns if any(x[0] == 'n' and x[1]
== key for x in bits_of(m, a.lhs).bits)]
                if len(da) == 1 and sub_ok(da[0].lhs): drv_a = da[0]
            asg_loads = [a for a in m.assigns if sub_ok(a.rhs) and
any(x[0] == 'n' and x[1] == key for x in bits_of(m, a.rhs).bits)]
            if to_pins:
                sel = []
                for nm, pn in to_pins:
                    hit = next((l for l in loads + cplx if l[0].name
== nm and l[1] == pn), None)
                    if hit is None: raise EcoError('Pin %s/%s is not a
load of net %s.' % (nm, pn, key))
                    sel.append(hit)
                moved = sel; all_loads = False
            else: moved = list(loads); all_loads = True
            drv_simple = drv is not None and any(c.pin == drv[1] and
_simple_expr(c.expr) == key for c in drv[0].conns)
            mode_b = all_loads and (drv_simple or drv_c is not None or
drv_a is not None)   # driver side: chain drives the original net, so
ports/complex loads stay put
            moved_a = []
            if all_loads and not mode_b:               # bus /
concatenation / assign loads follow the chain too: only the one bit is
swapped
                moved = moved + [x for x in cplx if
sub_ok(conn_expr(x[0], x[1]))]; moved_a = asg_loads
            if not mode_b and not moved and not moved_a:
                raise EcoError('Net %s has no loads that can be moved
behind the chain%s.' % (key, ' (only an output port, and no driver
that can be re-pointed)' if (port and port.dir != 'input') else ''))
            if all_loads and not mode_b and (port and port.dir != 'input'):
                log.append('  note: the output port %s stays on the
original net' % port.name)
            taken = set(); wid = 1
            def netname(k):
                nm = _fresh(m, ('%s_net_%d' if auto_nets else
'%s_w%d') % (prefix, k), taken); taken.add(nm); return nm
            insts = []
            for k in range(count):
                nm = _fresh(m, '%s_%s_%d' % (prefix, tag, k + 1),
taken); taken.add(nm); insts.append(nm)
            nets = [netname(k + 1) for k in range(count)]
            for nn in nets: add_net(m, nn)
            if mode_b:
                # driver -> nets[0] -> inst1 -> nets[1] ... -> inst_n
-> original net
                if drv_simple: ensure_named(D, self.lib, drv[0]);
set_conn(m, drv[0], drv[1], nets[0])
                elif drv_c is not None: ensure_named(D, self.lib,
drv_c[0]); set_conn(m, drv_c[0], drv_c[1], subst_bit(m,
conn_expr(drv_c[0], drv_c[1]), key, nets[0]))
                else: jobj(drv_a); drv_a.lhs = subst_bit(m, drv_a.lhs,
key, nets[0]); touch(m)
                chain_in = [nets[k] for k in range(count)]; chain_out
= [nets[k + 1] for k in range(count - 1)] + [key]
            else:
                chain_in = [key] + nets[:-1]; chain_out = nets
            for k, nm in enumerate(insts):
                i = new_inst_named(m, cell, nm); set_conn(m, i, ipin,
chain_in[k]); set_conn(m, i, opin, chain_out[k])
            if not mode_b:
                for inst, pin in moved:
                    ensure_named(D, self.lib, inst)
                    if (id(inst), pin) in cplx_set: set_conn(m, inst,
pin, subst_bit(m, conn_expr(inst, pin), key, nets[-1]))
                    else: set_conn(m, inst, pin, nets[-1])
            for a_ in moved_a: jobj(a_); a_.rhs = subst_bit(m, a_.rhs,
key, nets[-1]); touch(m)
            # ---- post-condition: the chain really joins the driver
side to every moved load ----
            def uses(inst, pin, nn): return any(x[0] == 'n' and x[1]
== nn for x in bits_of(m, conn_expr(inst, pin) or '').bits)
            if mode_b:
                ok_drv = (conn_expr(drv[0], drv[1]) == nets[0]) if
drv_simple else (uses(drv_c[0], drv_c[1], nets[0]) if drv_c is not
None else any(x[0] == 'n' and x[1] == nets[0] for x in bits_of(m,
drv_a.lhs).bits))
                if not ok_drv or conn_expr(m.inst(insts[-1]), opin) !=
key: raise EcoError('eco_add_repeater: internal check failed (driver
side not joined); the edit was rolled back.')
            else:
                for inst, pin in moved:
                    if not uses(inst, pin, nets[-1]): raise
EcoError('eco_add_repeater: internal check failed (%s/%s is not on the
last stage); the edit was rolled back.' % (inst.name, pin))
                if conn_expr(m.inst(insts[0]), ipin) != key or
conn_expr(m.inst(insts[-1]), opin) != nets[-1]: raise
EcoError('eco_add_repeater: internal check failed (chain ends); the
edit was rolled back.')
            log.append('ECO: eco_add_repeater %s/%s: %d x %s on net %s
(%s)' % ('/'.join(path), prefix, count, cell, key, 'driver side' if
mode_b else '%d load(s) moved' % len(moved)))
            D.eco_log.extend(log); return {'insts': [self.hname(path,
n) for n in insts], 'nets': [self.hname(path, n) for n in nets]},
created
        res = self.do_edit('eco_add_repeater', [path], fn)
        return res[0]['insts'] if isinstance(res, tuple) else []

    s_eco_delete_repeater = {'-from_pin': 's', '-net': 's', '-stages':
'i', '-prefix': 's', '-force': 'b'}
    def c_eco_delete_repeater(self, o, pos):
        self.need_design()
        if ('-from_pin' in o) == ('-net' in o): raise
CmdError('eco_delete_repeater: -from_pin and -net are exclusive; give
exactly one\n' + USAGE['eco_delete_repeater'])
        if '-net' in o:
            path, m, base, key = self.resolve_net(o['-net'])
        else:
            ref = self.resolve_pin(o['-from_pin']); ks = self.pin_nets(ref)
            if ref['inst'] is None or len(ks) != 1: raise
CmdError('eco_delete_repeater: -from_pin needs an instance pin on one
simple net bit.')
            path, key = ks[0]; m = ref['m']
        D = self.D; lib = self.lib; limit = o.get('-stages', 10 ** 9);
pfx = o.get('-prefix')
        eco_rx = re.compile(r'_(buf|inv)_\d+$')
        def probe(pth, k):
            mm0 = self.mod_at(pth); d0, l0, c0, p0 = self.net_loads(mm0, k, pth)
            if len(l0) != 1 or c0 or (p0 and p0.dir == 'output'): return False
            inf = repeater_info(lib, l0[0][0].type)
            return inf is not None and inf[0] == l0[0][1]
        if '-from_pin' in o:                      # climb to the
hierarchy level where the chain lives
            while not probe(path, key) and path:
                mm0 = self.mod_at(path); base = key.split('[')[0]; pn
= mm0.nets.get(base)
                if pn is None or not pn.dir: break
                par = self.mod_at(path[:-1]).inst(path[-1]); ce =
conn_expr(par, base) if par is not None else None
                if not ce or not SIMPLE_SIG.match(ce.strip()): break
                path = path[:-1]; key = _simple_expr(ce)
        def fn():
            log = []; created = []
            mm = self.modules_for(path, log, created)[-1]; chain = [];
cur = key; nets = [key]
            while len(chain) < limit:
                drv, loads, cplx, port = self.net_loads(mm, cur, path)
                if len(loads) != 1 or cplx or (port and port.dir !=
'input' and chain) or (port and port.dir == 'output'): break
                inst, pin = loads[0]; info = repeater_info(lib, inst.type)
                if info is None or info[0] != pin: break
                if pfx and not inst.name.startswith(pfx): break
                if not pfx and '-force' not in o and not
eco_rx.search(inst.name): break   # never remove design buffers
implicitly
                oexpr = conn_expr(inst, info[1])
                if not oexpr or not SIMPLE_SIG.match(oexpr): break
                chain.append((inst, info)); cur = _simple_expr(oexpr);
nets.append(cur)
            if not chain: raise EcoError('No ECO repeater chain starts
at %s (only eco_add_repeater-named stages are removed; use -prefix <p>
or -force for others).' % key)
            inv = sum(1 for _, i in chain if i[2])
            if inv % 2 and '-force' not in o:
                raise EcoError('The chain of %d stage(s) has %d
inverter(s): removing it would invert the signal. Use -stages to
remove an even count, or -force.' % (len(chain), inv))
            last = nets[-1]; drv0, _, _, _ = self.net_loads(mm, key, path)
            _, lloads, lcplx, lport = self.net_loads(mm, last, path)
            if lport is None and not lcplx:
                for inst, pin in lloads: ensure_named(D, lib, inst);
set_conn(mm, inst, pin, key)
                drop = nets[1:]
            else:
                if drv0 is None: raise EcoError('Cannot remove the
chain: its output net %s is a port or complex expression and %s has no
instance driver.' % (last, key))
                ensure_named(D, lib, drv0[0]); set_conn(mm, drv0[0],
drv0[1], last); drop = nets[:-1]
            for inst, _ in chain: remove_instance(mm, inst.name);
mm.layout and mm.layout.pop('i:' + inst.name, None)
            for nn in drop:
                n = mm.nets.get(nn)
                if n is not None and not n.dir and not
any(self.net_loads(mm, k2, path)[0:3] != (None, [], []) for k2 in
([nn] if not n.vec else ['%s[%d]' % (nn, b) for b in range(min(n.msb,
n.lsb), max(n.msb, n.lsb) + 1)])) and not any(nn in a.lhs or nn in
a.rhs for a in mm.assigns): remove_net(mm, nn)
            log.append('ECO: eco_delete_repeater /%s: removed %d
stage(s) %s' % ('/'.join(path), len(chain), ', '.join(i.name for i, _
in chain)))
            D.eco_log.extend(log); return [self.hname(path, i.name)
for i, _ in chain], created
        res = self.do_edit('eco_delete_repeater', [path], fn)
        return res[0] if isinstance(res, tuple) else []

    def need_proj(self):
        if not self.app.proj.cfg: raise CmdError('no flow project: use
flow_load_config <config.mk> first')
    s_flow_load_config = {'-flow_home': 's'}
    def c_flow_load_config(self, o, pos):
        pp = self.flat(pos)
        if not pp: raise CmdError('flow_load_config: config file
required\n' + USAGE['flow_load_config'])
        if '-flow_home' in o: self.app.flow_prefs['flow_home'] = o['-flow_home']
        if not os.path.isfile(pp[0]): raise
CmdError('flow_load_config: file not found: ' + pp[0])
        if self.app.flow_open_config(pp[0]) is None: raise
CmdError('flow_load_config: cannot read ' + pp[0])
        return self.app.proj.val('DESIGN_NAME')
    s_flow_run = {'-stage': 'l', '-wait': 'b', '-no_load': 'b'}
    def c_flow_run(self, o, pos):
        self.need_proj(); a = self.app; names = [s[0] for s in ORFS_STAGES]
        st = o.get('-stage', []) + self.flat(pos)
        if not st or 'all' in st: st = names
        for x in st:
            if x not in names and x != 'clean': raise
CmdError('flow_run: unknown stage "%s" (use %s, all or clean)' % (x,
', '.join(names)))
        if '-no_load' in o: a.flow_auto.set(False)
        if not a.flow_run(st): raise CmdError('flow_run: could not
start (see the Flow tab)')
        if '-wait' in o:
            a.flow_wait()
            if getattr(a, 'flow_rc', 0): raise CmdError('flow_run:
stage %s failed (exit code %s); see the Flow tab' % (a.flow_cur,
a.flow_rc))
            bad = [x for x in st if x != 'clean' and not a.proj.stage_done(x)]
            if bad: raise CmdError('flow_run: no results for: ' + ' '.join(bad))
        return st
    def c_flow_stop(self, o, pos): self.app.flow_stop(); return 'stopped'
    def c_flow_status(self, o, pos):
        self.need_proj(); p = self.app.proj
        return ['config %s' % p.cfg, 'design %s' %
p.val('DESIGN_NAME'), 'platform %s' % p.val('PLATFORM'), 'flow_home
%s' % p.flow_home, 'running %s' % (self.app.flow_cur if
self.app.flow_proc is not None else 'no')] + ['%s %s' % (s[0], 'done'
if p.stage_done(s[0]) else '-') for s in ORFS_STAGES]
    def c_flow_get_var(self, o, pos):
        self.need_proj(); pp = self.flat(pos)
        if not pp: raise CmdError('flow_get_var: variable name required')
        return self.app.proj.val(pp[0])
    def c_flow_set_var(self, o, pos):
        self.need_proj(); pp = self.flat(pos)
        if len(pp) < 2: raise CmdError('flow_set_var: <NAME> <value> required')
        self.app.proj.over[pp[0]] = ' '.join(pp[1:]); return pp[0]
    s_flow_load_netlist = {'-wait': 'b'}
    def c_flow_load_netlist(self, o, pos):
        self.need_proj(); self.app.flow_load_netlist()
        if '-wait' in o: self.app.flow_wait()
        return self.app.top or ''
    s_flow_show_layout = {'-stage': 's', '-wait': 'b'}
    def c_flow_show_layout(self, o, pos):
        self.need_proj(); self.app.flow_show_layout(o.get('-stage'))
        if '-wait' in o: self.app.flow_wait()
        lp = self.app.layout
        return ['%d cells' % len(lp.D.comps), '%d pins' %
len(lp.D.pins)] if lp.D else []
    s_clone_inst = {'-inst': 'l', '-num_clones': 'i', '-auto_tie': 's'}
    def c_clone_inst(self, o, pos):
        self.need_design(); pats = o.get('-inst', []) + self.flat(pos)
        if not pats: raise CmdError('clone_inst: -inst is required\n'
+ USAGE['clone_inst'])
        n = o.get('-num_clones', 1); tie = o.get('-auto_tie', '0') in
('1', 'true', 'yes')
        if n < 1: raise CmdError('clone_inst: -num_clones must be >= 1')
        tg = self.swap_targets(pats); D = self.D; lib = self.lib
        def fn():
            log = []; created = []; out = []
            for path, inst in tg:
                m = self.modules_for(path, log, created)[-1]; src =
m.inst(inst.name)
                rt, pins = self.pins_of(src); ensure_named(D, lib,
src); pdm = {p.name: p for p in pins}
                for k in range(n):
                    nm = _fresh(m, '%s_clone_%d' % (src.name, k + 1));
c = new_inst_named(m, src.type, nm)
                    c.params = src.params; c.attrs = src.attrs
                    have = set()
                    for cn in src.conns:
                        pd = pdm.get(cn.pin); have.add(cn.pin)
                        if pd is not None and pd.dir == 'output':
continue          # outputs stay open
                        set_conn(m, c, cn.pin, cn.expr)
                    if tie:
                        for pd in pins:
                            if pd.dir == 'input' and (pd.name not in
have or not conn_expr(src, pd.name)):
                                set_conn(m, c, pd.name, ("%d'b0" % 1)
if pd.width <= 1 else "%d'b%s" % (pd.width, '0' * pd.width))
                    out.append(self.hname(path, nm)); log.append('ECO:
clone_inst %s -> %s' % (self.hname(path, src.name), out[-1]))
            D.eco_log.extend(log); return out, created
        res = self.do_edit('clone_inst', [p for p, _ in tg], fn)
        return res[0] if isinstance(res, tuple) else []

    # ---- selection ----
    def found_for(self, kind, names):
        out = []
        for n in names:
            segs = n.split('/')
            if kind == 'inst': out.append(Found('inst',
tuple(segs[:-1]), segs[-1], ''))
            elif kind == 'pin': out.append(Found('pin',
tuple(segs[:-2]), segs[-2], segs[-1]) if len(segs) > 1 else
Found('topport', (), segs[-1], ''))
            elif kind == 'net': out.append(Found('net',
tuple(segs[:-1]), self._split_bit(segs[-1])[0], ''))
        return out

    def _select(self, kind, names, o):
        objs = self.found_for(kind, names)
        a = self.app
        cur = a.selection_found()
        if '-add' in o and cur: objs = list(cur) + [x for x in objs if
x not in cur]
        a.search_root = a.root_mod; a.set_msel(objs)
        if '-zoom' in o and objs: a.zoom_to(objs)
        return names

    s_select_inst = {'-add': 'b', '-zoom': 'b'}
    def c_select_inst(self, o, pos):
        self.need_design(); names = []
        for p in self.flat(pos):
            for path, inst in self.match_insts(p, '/' not in p):
                n = self.hname(path, inst.name)
                if n not in names: names.append(n)
        return self._select('inst', names, o)

    s_select_pin = {'-regexp': 'l', '-add': 'b', '-zoom': 'b'}
    def c_select_pin(self, o, pos):
        self.need_design(); pats = o.get('-regexp', []) + self.flat(pos)
        if not pats: raise CmdError('select_pin: -regexp <pattern> is
required\n' + USAGE['select_pin'])
        names = self.c_get_pins({'-hier': True} if any('/' not in p
for p in pats) else {}, pats)
        return self._select('pin', names, o)

    s_select_net = {'-add': 'b', '-zoom': 'b', '-hier': 'b'}
    def c_select_net(self, o, pos):
        self.need_design(); names = []
        for p in self.flat(pos):
            for n in self.net_names(p, '/' not in p):
                if n not in names: names.append(n)
        return self._select('net', names, o)

    s_deselect = {'-all': 'b'}
    def c_deselect(self, o, pos):
        a = self.app
        if '-all' in o or not self.flat(pos): a.clear_highlights();
a.select_none(); return 'deselected all'
        pats = self.flat(pos); rxs = [glob_rx(p.lstrip('/')) for p in pats]
        keep = [f for f in a.selection_found() if not
any(r.match(found_str(f).lstrip('/')) for r in rxs)]
        n0 = len(a.selection_found()); a.select_none();
a.set_msel(keep); return 'deselected %d' % (n0 - len(keep))

    s_set_eco_option = {'-uniquify': 's', '-threads': 'i', '-lib_check': 's'}
    def c_set_eco_option(self, o, pos):
        if '-uniquify' in o:
            if o['-uniquify'] not in ('yes', 'no', 'ask'): raise
CmdError('set_eco_option: -uniquify yes|no|ask')
            self.uniq = o['-uniquify']
        if '-threads' in o:
            if o['-threads'] < 0: raise CmdError('set_eco_option:
-threads N (0 = all cores)')
            THREADS[0] = o['-threads']
        if '-lib_check' in o:
            if o['-lib_check'] not in ('strict', 'warn', 'off'): raise
CmdError('set_eco_option: -lib_check strict|warn|off')
            self.app.lib_check = o['-lib_check']
            try: self.app.v_libchk.set(o['-lib_check'])
            except Exception: pass
        return 'uniquify = %s, threads = %d of %d cores, lib_check =
%s%s' % (self.uniq, n_workers(), os.cpu_count() or 1,
self.app.lib_check, '' if fork_ok() else ' (parallel workers need
Linux/macOS)')

    s_check_netlist_cells = {'-missing_only': 'b'}
    def c_check_netlist_cells(self, o, pos):
        self.need_design(); used, missing, nofp =
collect_leaf_cells(self.D, self.lib)
        rows = ['%s  %d  %s' % (t, n, 'MISSING' if t in missing else
('no-footprint' if t in nofp else 'ok')) for t, n in
sorted(used.items(), key=lambda x: -x[1]) if not
(o.get('-missing_only') and t not in missing)]
        return rows

    def c_help(self, o, pos):
        a = self.flat(pos)
        if a and a[0] in USAGE: return USAGE[a[0]]
        return 'Commands (each accepts -help):\n  ' + '\n
'.join(textwrap.wrap(', '.join(sorted(k for k in USAGE if k not in
('source',)) + ['source']), 96)) + '\nTcl and Python share the
commands. Name patterns accept * and ?.'

    # ================= language front ends =================
    def install_tcl(self):
        tk = self.tkc
        def call(name, *args):
            try:
                r = self.run(name, list(args))
                if isinstance(r, (list, tuple)): r =
tcl_join(tuple(r)) if r else ''
                return (0, '' if r is None else str(r))
            except (CmdError, EcoError) as ex: return (1, str(ex))
            except Exception as ex:
                log_crash(traceback.format_exc()); return (1,
'internal error: %s: %s' % (type(ex).__name__, ex))
        tk.createcommand('__ns_call', lambda *a: tcl_join(tuple(str(x)
for x in call(*a))))
        def out(s, nl=1): self.out(str(s), bool(int(nl)))
        tk.createcommand('__ns_out', out)
        def src(path):
            return self.source_file(path)
        tk.createcommand('__ns_source', lambda p:
tcl_join(tuple(str(x) for x in src(p))))
        tk.createcommand('__ns_batch', lambda f:
(self.app.batch_begin() if int(f) else self.app.batch_end()) or '')
        script = '''
proc __ns_wrap {name args} {
    lassign [__ns_call $name {*}$args] rc res
    if {$rc} { return -code error $res }
    return $res
}
if {[info commands ::tcl_puts] eq ""} {
    rename puts ::tcl_puts
    proc puts {args} {
        set nl 1
        if {[lindex $args 0] eq "-nonewline"} { set nl 0; set args
[lrange $args 1 end] }
        if {[llength $args] == 2} {
            set ch [lindex $args 0]
            if {$ch ne "stdout" && $ch ne "stderr"} { if {$nl} {
return [tcl_puts $ch [lindex $args 1]] } else { return [tcl_puts
-nonewline $ch [lindex $args 1]] } }
            set s [lindex $args 1]
        } else { set s [lindex $args 0] }
        __ns_out $s $nl
    }
}
if {[info commands ::tcl_source] eq ""} {
    rename source ::tcl_source
    proc source {args} {
        set f [lindex $args end]
        if {[string match -nocase *.py $f]} { lassign [__ns_source $f]
rc res; if {$rc} { return -code error $res }; return $res }
        __ns_batch 1
        set rc [catch {uplevel 1 [list ::tcl_source {*}$args]} res opts]
        __ns_batch 0
        return -options $opts $res
    }
}
'''
        tk.eval(script)
        for n in self.cmds:
            if n == 'source': continue
            tk.eval('proc %s {args} { __ns_wrap %s {*}$args }' % (n, n))

    def py_namespace(self):
        ns = {'app': self.app, 'shell': self}
        def mk(n):
            def f(*args):
                r = self.run(n, list(args))
                if isinstance(r, str) and (n.startswith('report') or
'-help' in args or n in ('help', 'set_eco_option')): self.out(r)
                return r
            f.__name__ = n; f.__doc__ = USAGE[n]; return f
        for n in self.cmds:
            if n != 'source': ns[n] = mk(n)
        ns['source'] = lambda p: self.source_file(p, True)
        ns['puts'] = lambda *a: self.out(' '.join(str(x) for x in a))
        ns['re'] = re; ns['ns'] = sys.modules[__name__]
        return ns

    def source_file(self, path, raw=False):
        """run a .tcl or .py command file as one batch (the view
refreshes once at the end)"""
        path = os.path.expanduser(path)
        if not os.path.isfile(path):
            msg = 'couldn\'t read file "%s": no such file' % path
            if raw: raise CmdError(msg)
            return (1, msg)
        try:
            if path.lower().endswith('.py'):
                self.app.batch_begin()
                try:
                    with open(path) as f: code = compile(f.read(), path, 'exec')
                    self.pyns['D'] = self.D; self.pyns['lib'] =
self.lib; self.pyns['__file__'] = path; exec(code, self.pyns)
                finally: self.app.batch_end()
                return None if raw else (0, '')
            r = str(self.tkc.eval('source {%s}' % path))
            return r if raw else (0, r)
        except Exception as ex:
            if raw: raise
            return (1, '%s' % ex)

    # ---- console execution ----
    def exec_tcl(self, line):
        try: r = self.tkc.eval(line)
        except tk.TclError as ex: self.out('Error: %s' % ex); return
        if r:
            if len(r) > 400000: r = r[:400000] + '\n... (%d more
characters not shown; assign the command to a variable or write it to
a file)' % (len(r) - 400000)
            self.out(r)

    def exec_py(self, line):
        import contextlib, io
        self.pyns['D'] = self.D; self.pyns['lib'] = self.lib
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                try: code = compile(line, '<console>', 'eval'); r =
eval(code, self.pyns);
                except SyntaxError: exec(compile(line, '<console>',
'exec'), self.pyns); r = None
            if buf.getvalue(): self.out(buf.getvalue().rstrip('\n'))
            if r is not None: self.out(repr(r) if not isinstance(r, str) else r)
        except Exception as ex:
            if buf.getvalue(): self.out(buf.getvalue().rstrip('\n'))
            self.out('%s: %s' % (type(ex).__name__, ex))


class FormDialog(tk.Toplevel):
    """Small modal form: fields = [(key, label, default, options_or_None)]"""
    def __init__(self, parent, title, fields, ok='OK'):
        super().__init__(parent); self.title(title);
self.transient(parent); self.result = None; self.vars = {}
        f = ttk.Frame(self, padding=12); f.pack(fill='both', expand=True)
        first = None
        for r, (key, label, default, opts) in enumerate(fields):
            ttk.Label(f, text=label).grid(row=r, column=0, sticky='w',
padx=(0, 8), pady=4)
            var = tk.StringVar(value=default); self.vars[key] = var
            w = ttk.Combobox(f, textvariable=var, values=opts,
width=28) if opts else ttk.Entry(f, textvariable=var, width=30)
            w.grid(row=r, column=1, sticky='ew', pady=4); first = first or w
        b = ttk.Frame(f); b.grid(row=len(fields), column=0,
columnspan=2, sticky='e', pady=(10, 0))
        ttk.Button(b, text='Cancel',
command=self.destroy).pack(side='right', padx=(6, 0))
        ttk.Button(b, text=ok, command=self.accept).pack(side='right')
        self.bind('<Return>', lambda e: self.accept());
self.bind('<Escape>', lambda e: self.destroy())
        self.update_idletasks()
        self.geometry('+%d+%d' % (parent.winfo_rootx() + 120,
parent.winfo_rooty() + 100))
        first.focus_set(); self.grab_set(); self.wait_window(self)
    def accept(self): self.result = {k: v.get() for k, v in
self.vars.items()}; self.destroy()


class FindDialog(tk.Toplevel):
    KINDS = [('inst', 'Instances (hierarchical)'), ('pin', 'Pins
(hierarchical)'), ('net', 'Nets / wires (hierarchical)'),
             ('port', 'Module ports (hierarchical)'), ('topport',
'Top-module I/O ports')]
    SCOPES = [('all', 'Whole hierarchy from the top'), ('below',
'Current module and everything below'), ('module', 'Current module
only')]
    HINTS = {'inst': 'matches /u_core/u_alu/U12', 'pin': 'matches
/u_core/u_alu/U12/ZN', 'net': 'matches /u_core/u_alu/n123',
             'port': 'matches /u_core/u_alu/data_in  (a port of module
instance u_alu)', 'topport': 'matches the port name, e.g. clk or
^data'}
    def __init__(self, app, kind='inst'):
        super().__init__(app.root); self.app = app; self.title('Find
objects by regular expression'); self.results = [];
self.geometry('+%d+%d' % (app.root.winfo_rootx() + 120,
app.root.winfo_rooty() + 90))
        self.transient(app.root); f = ttk.Frame(self, padding=10);
f.pack(fill='both', expand=True)
        r = ttk.Frame(f); r.pack(fill='x'); ttk.Label(r, text='Object
type', width=12).pack(side='left')
        self.kind_var = tk.StringVar(); cb = ttk.Combobox(r,
textvariable=self.kind_var, state='readonly', width=34, values=[k[1]
for k in self.KINDS]); cb.pack(side='left');
cb.bind('<<ComboboxSelected>>', lambda e: self.on_kind())
        r2 = ttk.Frame(f); r2.pack(fill='x', pady=4); ttk.Label(r2,
text='Regexp', width=12).pack(side='left')
        self.rx = tk.StringVar(); self.entry = ttk.Entry(r2,
textvariable=self.rx, width=52, font=(app.mono, 10));
self.entry.pack(side='left', fill='x', expand=True)
        ttk.Button(r2, text='Search',
command=self.search).pack(side='left', padx=(6, 0))
        self.hint = ttk.Label(f, text='', style='Hint.TLabel');
self.hint.pack(anchor='w')
        r3 = ttk.Frame(f); r3.pack(fill='x', pady=4)
        self.icase = tk.BooleanVar(value=True); self.local =
tk.BooleanVar(value=False)
        ttk.Checkbutton(r3, text='Ignore case',
variable=self.icase).pack(side='left'); ttk.Checkbutton(r3,
text='Match local name only', variable=self.local).pack(side='left',
padx=10)
        self.scope_var = tk.StringVar(value=self.SCOPES[0][1]);
ttk.Combobox(r3, textvariable=self.scope_var, state='readonly',
width=34, values=[s[1] for s in self.SCOPES]).pack(side='left',
padx=6)
        tw = ttk.Frame(f); tw.pack(fill='both', expand=True, pady=4)
        self.tree = ttk.Treeview(tw, columns=('kind', 'mod'),
show='tree headings', height=14, selectmode='extended');
self.tree.heading('#0', text='Hierarchical name');
self.tree.heading('kind', text='Type'); self.tree.heading('mod',
text='In module')
        self.tree.column('#0', width=430); self.tree.column('kind',
width=70); self.tree.column('mod', width=140)
        ys = ttk.Scrollbar(tw, orient='vertical',
command=self.tree.yview); self.tree.configure(yscrollcommand=ys.set);
self.tree.pack(side='left', fill='both', expand=True);
ys.pack(side='right', fill='y')
        self.tree.bind('<Double-Button-1>', self.on_double)
        self.status = ttk.Label(f, text='Type a regular expression and
press Enter.', style='Hint.TLabel'); self.status.pack(anchor='w')
        b = ttk.Frame(f); b.pack(fill='x', pady=(6, 0))
        for txt, cmd in (('Select', self.do_select), ('Highlight',
self.do_highlight), ('Zoom to', self.do_zoom), ('Select + zoom',
lambda: (self.do_select(), self.do_zoom())), ('Clear highlights',
app.clear_highlights)):
            ttk.Button(b, text=txt, command=cmd).pack(side='left', padx=2)
        ttk.Button(b, text='Close', command=self.destroy).pack(side='right')
        ttk.Label(f, text='Select / Highlight / Zoom act on the
highlighted rows, or on all results when none are highlighted.',
style='Hint.TLabel').pack(anchor='w', pady=(4, 0))
        self.entry.bind('<Return>', lambda e: self.search());
self.bind('<Escape>', lambda e: self.destroy()); self.set_kind(kind);
self.entry.focus_set()
    def kind(self):
        v = self.kind_var.get()
        for k, t in self.KINDS:
            if t == v: return k
        return 'inst'
    def scope(self):
        v = self.scope_var.get()
        for k, t in self.SCOPES:
            if t == v: return k
        return 'all'
    def set_kind(self, kind):
        for k, t in self.KINDS:
            if k == kind: self.kind_var.set(t)
        self.on_kind(); self.entry.focus_set()
    def on_kind(self): self.hint.configure(text='Regexp %s' %
self.HINTS[self.kind()])
    def search(self):
        app = self.app; kind = self.kind(); pat = self.rx.get()
        if not pat.strip(): self.status.configure(text='Enter a
regular expression.'); return
        try: re.compile(pat)
        except re.error as ex: self.status.configure(text='Invalid
regexp: %s' % ex); return
        root = app.root_mod or app.top; sc = self.scope(); cur =
app.cur_path(); t0 = time.time()
        icase = bool(self.icase.get()); local = bool(self.local.get())
         # read Tk variables here, on the main thread
        def work(prog, cancel): return find_objects(app.D, app.lib,
root, kind, pat, icase, sc, cur, local, 20000, prog, cancel)
        def done(res):
            self.results, trunc = res; app.search_root = root;
self.show(trunc, time.time() - t0)
        app.run_job('Searching %s' % dict(self.KINDS)[kind].lower(), work, done)
    def show(self, trunc, dt):
        tv = self.tree; tv.delete(*tv.get_children()); names = dict(self.KINDS)
        for k, f in enumerate(self.results[:3000]):
            mod = ''
            try: mod = chain_modules(self.app.D, self.app.search_root,
f.path)[-1].name
            except EcoError: pass
            tv.insert('', 'end', iid=str(k), text=found_str(f),
values=({'inst': 'instance', 'pin': 'pin', 'net': 'net', 'port':
'port', 'topport': 'top port'}[f.kind], mod))
        n = len(self.results); self.status.configure(text='%d
match%s%s in %.1f s%s' % (n, '' if n == 1 else 'es', ' (list limited
to the first 20,000)' if trunc else '', dt, ('; showing the first 3000
rows' if n > 3000 else '')))
        self.app.found_list = list(self.results); self.app.found_idx = -1
    def objs(self):
        sel = self.tree.selection()
        if sel: return [self.results[int(i)] for i in sel]
        return list(self.results)
    def do_select(self):
        o = self.objs()
        if o: self.app.search_root = self.app.search_root or
self.app.root_mod; self.app.set_msel(o)
    def do_highlight(self):
        o = self.objs()
        if o: self.app.add_highlight(o)
    def do_zoom(self):
        o = self.objs()
        if o: self.app.zoom_to(o)
    def on_double(self, e):
        iid = self.tree.identify_row(e.y)
        if iid: f = self.results[int(iid)]; self.app.set_msel([f]);
self.app.zoom_to([f])


# ====================================================================
#  OpenROAD-flow-scripts (ORFS) integration:
#  config.mk project files, flow runner, DEF/LEF layout viewer
# ====================================================================
ORFS_STAGES = [('synth', 'Synthesis (Yosys)', 'synth', 1),
('floorplan', 'Floorplan', 'floorplan', 2), ('place', 'Placement',
'place', 3),
               ('cts', 'Clock tree', 'cts', 4), ('route', 'Routing',
'route', 5), ('final', 'Finish', 'finish', 6)]
ORFS_GUI_TARGET = {'floorplan': 'gui_floorplan', 'place': 'gui_place',
'cts': 'gui_cts', 'route': 'gui_route', 'final': 'gui_final'}
CFG_FILE = os.path.join(os.path.expanduser('~'), '.netlist_studio_flow.json')


def _mk_cond(expr, vars_, expand):
    """evaluate the condition of ifeq / ifneq / ifdef / ifndef"""
    kw, _, rest = expr.strip().partition(' '); rest = rest.strip()
    if kw in ('ifdef', 'ifndef'):
        d = bool(expand(vars_.get(rest.strip(), ''), vars_)) if
rest.strip() in vars_ else False
        return d if kw == 'ifdef' else not d
    m = re.match(r'^\(\s*(.*?)\s*,\s*(.*?)\s*\)$', rest) or
re.match(r'^(["\'])(.*?)\1\s+(["\'])(.*?)\3$', rest)
    if not m: return True
    a, b = (m.group(1), m.group(2)) if m.re.pattern.startswith('^\\(')
else (m.group(2), m.group(4))
    eq = expand(a, vars_).strip() == expand(b, vars_).strip()
    return eq if kw == 'ifeq' else not eq


_MK_FN = re.compile(r'\$[({]([^(){}$]*)[)}]')


def mk_expand(raw, vars_, depth=0):
    """expand $(VAR) / ${VAR} and a few make functions (wildcard dir
notdir basename abspath strip subst patsubst)"""
    if raw is None: return ''
    s = raw
    for _ in range(40):
        def sub(m):
            t = m.group(1).strip(); fn, _, arg = t.partition(' ')
            if arg and fn in ('wildcard', 'dir', 'notdir', 'basename',
'abspath', 'realpath', 'strip', 'subst', 'patsubst', 'shell',
'firstword', 'addprefix', 'addsuffix', 'filter', 'filter-out', 'sort',
'words', 'if'):
                a = arg.strip(); w = a.split()
                try:
                    if fn == 'wildcard': return ' '.join(sorted(x for
p in w for x in glob.glob(p)))
                    if fn == 'dir': return '
'.join((os.path.dirname(x) + '/') if os.path.dirname(x) else './' for
x in w)
                    if fn == 'notdir': return '
'.join(os.path.basename(x) for x in w)
                    if fn == 'basename': return '
'.join(os.path.splitext(x)[0] for x in w)
                    if fn in ('abspath', 'realpath'): return '
'.join(os.path.abspath(x) for x in w)
                    if fn == 'strip': return ' '.join(w)
                    if fn == 'firstword': return w[0] if w else ''
                    if fn == 'sort': return ' '.join(sorted(set(w)))
                    if fn == 'words': return str(len(w))
                    if fn in ('filter', 'filter-out'):
                        p = a.split(',', 1)
                        if len(p) != 2: return a
                        pats = [re.compile('^' +
re.escape(x).replace('%', '.*') + '$') for x in p[0].split()]
                        return ' '.join(x for x in p[1].split() if
any(r.match(x) for r in pats) == (fn == 'filter'))
                    if fn == 'subst':
                        p = [x.strip() for x in a.split(',', 2)];
return p[2].replace(p[0], p[1]) if len(p) == 3 else a
                    if fn == 'patsubst':
                        p = [x.strip() for x in a.split(',', 2)]
                        if len(p) != 3: return a
                        rx = re.compile('^' +
re.escape(p[0]).replace('%', '(.*)') + '$')
                        return ' '.join((rx.sub(lambda mm:
p[1].replace('%', mm.group(1) if mm.groups() else ''), x)) for x in
p[2].split())
                    if fn == 'addprefix': p = a.split(',', 1); return
' '.join(p[0].strip() + x for x in p[1].split()) if len(p) == 2 else a
                    if fn == 'addsuffix': p = a.split(',', 1); return
' '.join(x + p[0].strip() for x in p[1].split()) if len(p) == 2 else a
                    if fn == 'shell': return ''
                except Exception: return ''
                return ''
            v = vars_.get(t)
            if v is None: return os.environ.get(t, '')
            return v if not isinstance(v, tuple) else v[0]
        n = _MK_FN.sub(sub, s)
        if n == s: break
        s = n
    return s


def parse_mk(text, vars_=None, defined=None, base_dir='.',
only_new=False, depth=0):
    """Parse a make style config file (export VAR = value ...).  vars_
is updated in place with RAW values
    (expanded lazily); `defined` collects the names assigned by this
file.  Returns the list of (name, op, raw, line)."""
    vars_ = vars_ if vars_ is not None else {}; defined = defined if
defined is not None else set(); items = []
    lines = text.splitlines(); i = 0; stack = []        # stack of
[active, taken, parent_active]
    def active(): return all(f[0] for f in stack)
    while i < len(lines):
        ln = i + 1; s = lines[i]; i += 1
        while s.rstrip().endswith('\\') and i < len(lines): s =
s.rstrip()[:-1] + ' ' + lines[i].strip(); i += 1
        s = re.sub(r'(^|\s)#.*$', '', s).strip()
        if not s: continue
        m = re.match(r'^(ifeq|ifneq|ifdef|ifndef)\b(.*)$', s)
        if m:
            c = _mk_cond(s, vars_, mk_expand) if active() else False;
stack.append([c, c]); continue
        if re.match(r'^else\b', s) and stack:
            rest = s[4:].strip()
            if stack[-1][1]: stack[-1][0] = False
            else:
                c = _mk_cond(rest, vars_, mk_expand) if rest else
True; stack[-1][0] = c; stack[-1][1] = c
            continue
        if re.match(r'^endif\b', s):
            if stack: stack.pop()
            continue
        if not active(): continue
        m = re.match(r'^-?include\s+(.+)$', s)
        if m and depth < 8:
            for f in mk_expand(m.group(1), vars_).split():
                f = f if os.path.isabs(f) else os.path.join(base_dir, f)
                try: items += parse_mk(read_text(f), vars_, defined,
os.path.dirname(f), only_new, depth + 1)
                except OSError: pass
            continue
        m = re.match(r'^(?:export\s+|override\s+)*([A-Za-z_][A-Za-z0-9_.]*)\s*(\?=|::=|:=|\+=|=)\s*(.*)$',
s)
        if not m: continue
        name, op, raw = m.group(1), m.group(2), m.group(3).strip()
        if only_new and name in defined and name not in
getattr(parse_mk, '_own', set()): continue
        if op == '?=':
            if name in vars_ or name in os.environ: continue
        elif op == '+=':
            old = vars_.get(name, ''); raw = (old + ' ' + raw).strip()
if old else raw
        elif op in (':=', '::='): raw = mk_expand(raw, vars_)
        vars_[name] = raw; defined.add(name); items.append((name, op, raw, ln))
    return items


def find_tool(name, flow_home=''):
    env = os.environ.get(name.upper() + '_EXE')
    if env and os.path.exists(env): return env
    p = shutil.which(name)
    if p: return p
    for base in (flow_home, os.path.dirname(flow_home or '')):
        if not base: continue
        for sub in ('tools/install/%s/bin/%s' % ('OpenROAD' if name ==
'openroad' else 'yosys', name), 'tools/install/%s/bin/%s' % (name,
name)):
            c = os.path.join(base, sub)
            if os.path.exists(c): return c
    return None


class OrfsProject:
    """An ORFS design: config.mk (+ the platform config.mk it selects)
and where its results are"""
    def __init__(self):
        self.cfg = None; self.vars = {}; self.own = set();
self.flow_home = ''; self.over = {}; self.extra = []

    def locate_flow_home(self, cfg, hint=''):
        cands = [hint, os.environ.get('FLOW_HOME', ''),
(os.environ.get('ORFS_ROOT', '') and
os.path.join(os.environ['ORFS_ROOT'], 'flow'))]
        d = os.path.dirname(os.path.abspath(cfg))
        for _ in range(8):
            cands.append(d); cands.append(os.path.join(d, 'flow')); d
= os.path.dirname(d)
        for c in cands:
            if c and os.path.isfile(os.path.join(c, 'Makefile')) and
os.path.isdir(os.path.join(c, 'scripts')): return os.path.abspath(c)
        return ''

    def load(self, cfg, flow_home=''):
        cfg = os.path.abspath(cfg); text = read_text(cfg)
        self.cfg = cfg; self.flow_home = self.locate_flow_home(cfg,
flow_home) or flow_home
        fh = self.flow_home or os.path.dirname(cfg)
        v = {'FLOW_HOME': fh, 'DESIGN_HOME': fh + '/designs',
'WORK_HOME': fh, 'DESIGN_CONFIG': cfg, 'PLATFORM_DIR': fh +
'/platforms/$(PLATFORM)',
             'FLOW_VARIANT': 'base', 'DESIGN_NICKNAME': '$(DESIGN_NAME)',
             'RESULTS_DIR':
'$(WORK_HOME)/results/$(PLATFORM)/$(DESIGN_NICKNAME)/$(FLOW_VARIANT)',
             'LOG_DIR':
'$(WORK_HOME)/logs/$(PLATFORM)/$(DESIGN_NICKNAME)/$(FLOW_VARIANT)',
             'REPORTS_DIR':
'$(WORK_HOME)/reports/$(PLATFORM)/$(DESIGN_NICKNAME)/$(FLOW_VARIANT)',
             'OBJECTS_DIR':
'$(WORK_HOME)/objects/$(PLATFORM)/$(DESIGN_NICKNAME)/$(FLOW_VARIANT)'}
        own = set(); items = parse_mk(text, v, own, os.path.dirname(cfg))
        self.items = items; self.own = set(own)
        plat = mk_expand('$(PLATFORM_DIR)', v) + '/config.mk'
        if os.path.isfile(plat):                           # the
platform provides defaults for everything the design did not set
            pv = dict(v); pown = set()
            parse_mk(read_text(plat), pv, pown, os.path.dirname(plat))
            for k, val in pv.items():
                if k not in own and k not in ('FLOW_HOME',
'DESIGN_HOME', 'WORK_HOME', 'DESIGN_CONFIG', 'PLATFORM_DIR'): v[k] =
val
        self.vars = v; self.platform_cfg = plat if os.path.isfile(plat) else ''
        return items

    def val(self, name, default=''):
        if name in self.over: return self.over[name]
        if name not in self.vars:
            e = os.environ.get(name)
            return e if e is not None else default
        return mk_expand(self.vars[name], self.vars)
    def files(self, name): return [f for f in self.val(name).split() if f]
    def results_dir(self): return self.val('RESULTS_DIR')
    def logs_dir(self): return self.val('LOG_DIR')
    def reports_dir(self): return self.val('REPORTS_DIR')
    def design(self): return self.val('DESIGN_NICKNAME') or
self.val('DESIGN_NAME') or 'design'

    def make_cmd(self, target, jobs=0):
        cmd = ['make', '-C', self.flow_home, 'DESIGN_CONFIG=' + self.cfg]
        if jobs: cmd.append('NUM_CORES=%d' % jobs)
        cmd += ['%s=%s' % (k, v) for k, v in self.over.items()] +
list(self.extra) + [target]
        return cmd

    def find_result(self, stage, exts=('def', 'odb', 'v')):
        """newest-stage result file of a stage: synth -> netlist,
others -> .def or .odb"""
        n = dict((s[0], s[3]) for s in ORFS_STAGES).get(stage); rd =
self.results_dir()
        if not n or not os.path.isdir(rd): return None
        for ext in exts:
            fs = sorted(glob.glob(os.path.join(rd, '%d_*.%s' % (n,
ext))), key=lambda p: (os.path.basename(p) not in ('1_synth.v',
'6_final.def'), -os.path.getmtime(p)))
            fs = [f for f in fs if not f.endswith(('.sdc', '.spef'))]
            if stage == 'synth' and ext == 'v':
                pref = [f for f in fs if os.path.basename(f) in
('1_synth.v', '1_2_yosys.v')]
                if pref: return pref[0]
            if fs: return fs[0]
        return None
    def stage_done(self, stage): return self.find_result(stage) is not None


class FlowRun:
    """one `make` invocation, output streamed through a queue; cancel
kills the whole process group"""
    def __init__(self, cmd, cwd, env=None):
        self.cmd = cmd; self.q = queue.Queue(); self.rc = None;
self.t0 = time.time()
        try:
            self.p = subprocess.Popen(cmd, cwd=cwd, env=env or
os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
text=True, errors='replace', bufsize=1, start_new_session=True)
        except OSError as ex:
            self.p = None; self.rc = 127; self.q.put('Error: cannot
start %s: %s' % (cmd[0], ex)); self.q.put(None); return
        threading.Thread(target=self._pump, daemon=True).start()
    def _pump(self):
        try:
            for ln in self.p.stdout: self.q.put(ln.rstrip('\n'))
        finally:
            self.p.wait(); self.rc = self.p.returncode; self.q.put(None)
    def alive(self): return self.p is not None and self.p.poll() is None
    def cancel(self):
        if self.p is None: return
        try:
            os.killpg(self.p.pid, signal.SIGTERM)
        except (OSError, ProcessLookupError): return
        def reaper():
            try: self.p.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try: os.killpg(self.p.pid, signal.SIGKILL)
                except OSError: pass
        threading.Thread(target=reaper, daemon=True).start()


# ---------------- LEF / DEF ----------------
class LefInfo:
    def __init__(self): self.layers = []; self.macros = {}; self.dbu = 1000.0

def parse_lef(paths):
    L = LefInfo()
    for p in paths:
        try: t = read_text(p)
        except OSError: continue
        m = re.search(r'DATABASE\s+MICRONS\s+(\d+)', t)
        if m: L.dbu = float(m.group(1))
        for m in re.finditer(r'^LAYER\s+(\S+)\s*\n(.*?)^END\s+\1\b',
t, re.M | re.S):
            ty = re.search(r'\bTYPE\s+(\w+)', m.group(2))
            if ty and ty.group(1) in ('ROUTING', 'CUT') and not
any(x[0] == m.group(1) for x in L.layers):
L.layers.append((m.group(1), ty.group(1)))
        starts = [(m.start(), m.group(1)) for m in
re.finditer(r'^MACRO\s+(\S+)', t, re.M)]
        for k, (a, nm) in enumerate(starts):
            seg = t[a:a + 1800]; sz =
re.search(r'SIZE\s+([\d.eE+-]+)\s+BY\s+([\d.eE+-]+)', seg); cl =
re.search(r'CLASS\s+(\w+)(?:\s+(\w+))?', seg)
            if sz: L.macros[nm] = (float(sz.group(1)),
float(sz.group(2)), (cl.group(1) + (' ' + cl.group(2) if cl.group(2)
else '')) if cl else '')
    return L


class DefDesign:
    def __init__(self):
        self.dbu = 1000.0; self.die = (0, 0, 1, 1); self.comps = [];
self.pins = []; self.segs = {}; self.vias = []; self.nets = 0;
self.rows = 0; self.name = ''


_RT_TOK = re.compile(r'\(\s*(\*|-?\d+)\s+(\*|-?\d+)(?:\s+-?\d+)?\s*\)|(NEW)\b|(\+)|([A-Za-z_][\w$.\[\]/]*)|(-?\d+)')
_RT_END = re.compile(r'\+\s*(USE|SOURCE|ORIGINAL|PROPERTY|WEIGHT|FIXEDBUMP|FREQUENCY|ESTCAP|NONDEFAULTRULE|XTALK|SHIELD|VPIN|PATTERN|STYLE)\b')

def _parse_route(s, segs, vias, special=False):
    m = _RT_END.search(s)
    if m: s = s[:m.start()]
    layer = None; px = py = None
    for t in _RT_TOK.finditer(s):
        a, b, new, plus, word, num = t.groups()
        if new: layer = None; px = py = None
        elif a is not None:
            x = px if a == '*' else int(a); y = py if b == '*' else int(b)
            if x is None or y is None: continue
            if layer is not None and px is not None and (x != px or y != py):
                arr = segs.get((layer, special))
                if arr is None: arr = segs[(layer, special)] = array('i')
                arr.extend((px, py, x, y))
            px, py = x, y
        elif word:
            if word in ('ROUTED', 'FIXED', 'COVER', 'NOSHIELD',
'TAPER', 'MASK', 'RECT', 'VIRTUAL'): continue
            if layer is None: layer = word
            elif px is not None: vias.append((px, py))       # a via
name at the current point
        # plus / num: width values and separators are ignored


def parse_def(text, prog=None, cancel=None):
    D = DefDesign()
    m = re.search(r'DESIGN\s+(\S+)\s*;', text); D.name = m.group(1) if m else ''
    m = re.search(r'UNITS\s+DISTANCE\s+MICRONS\s+(\d+)', text); D.dbu
= float(m.group(1)) if m else 1000.0
    m = re.search(r'DIEAREA\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)',
text)
    if m: D.die = tuple(int(x) for x in m.groups())
    D.rows = len(re.findall(r'^ROW\s', text, re.M))
    def section(name):
        m = re.search(r'^%s\s+\d+\s*;(.*?)^END\s+%s' % (name, name),
text, re.M | re.S)
        return m.group(1) if m else ''
    if prog: prog(0.1, 'components')
    body = section('COMPONENTS')
    rx = re.compile(r'-\s+(\S+)\s+(\S+)[^;]*?\+\s*(PLACED|FIXED|COVER)\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*(\w+)\s*;')
    for m in rx.finditer(body):
        D.comps.append((m.group(1), m.group(2), int(m.group(4)),
int(m.group(5)), m.group(6), m.group(3)))
    if cancel and cancel(): return D
    if prog: prog(0.3, 'pins')
    body = section('PINS')
    for m in re.finditer(r'-\s+(\S+)\s+\+\s*NET\s+(\S+)([^;]*?)\+\s*(?:PLACED|FIXED|COVER)\s*\(\s*(-?\d+)\s+(-?\d+)\s*\)\s*(\w+)\s*;',
body):
        d = re.search(r'DIRECTION\s+(\w+)', m.group(3)); ly =
re.search(r'LAYER\s+(\S+)', m.group(3))
        D.pins.append((m.group(1), (d.group(1) if d else '').upper(),
int(m.group(4)), int(m.group(5)), ly.group(1) if ly else ''))
    for nm, special in (('NETS', False), ('SPECIALNETS', True)):
        if prog: prog(0.5 if not special else 0.85, nm.lower())
        body = section(nm); n = 0
        for m in re.finditer(r'-\s+(\S+)(.*?);', body, re.S):
            s = m.group(2); k = s.find('ROUTED')
            if k < 0: k = s.find('FIXED')
            if k < 0: n += 1; continue
            n += 1; _parse_route(s[k:], D.segs, D.vias, special)
            if n % 20000 == 0 and cancel and cancel(): return D
        if not special: D.nets = n
    return D


def _unesc(n): return re.sub(r'\\(.)', r'\1', n)


LAYER_COLORS = ['#3b6fd8', '#d8443b', '#2ca05a', '#c58a14', '#8a4fc7',
'#1798a8', '#d0589a', '#6b7a1b', '#e0702a', '#4b5563']


class LayoutPane(ttk.Frame):
    """Physical view of an ORFS result: die, rows of cells, pins and
routing, drawn from DEF + LEF"""
    CELL_COL = {'comb': '#9bb7e0', 'seq': '#e0a35a', 'buf': '#8fcf9a',
'macro': '#b79ad6', 'fill': '#d7dbe0'}
    def __init__(self, parent, app):
        super().__init__(parent); self.app = app; self.D = None;
self.lef = None; self.z = 1.0; self.vx = self.vy = 0.0; self.hl =
set(); self.after_id = None
        self.show = {'cells': tk.BooleanVar(value=True), 'route':
tk.BooleanVar(value=True), 'pins': tk.BooleanVar(value=True), 'power':
tk.BooleanVar(value=False), 'fill': tk.BooleanVar(value=False)}
        self.layer_on = {}; self.press = None
        bar = ttk.Frame(self, padding=(6, 3)); bar.pack(side='top', fill='x')
        self.stage = tk.StringVar(value='final')
        ttk.Label(bar, text='Stage').pack(side='left')
        ttk.Combobox(bar, textvariable=self.stage, values=[s[0] for s
in ORFS_STAGES[1:]], state='readonly', width=10).pack(side='left',
padx=4)
        ttk.Button(bar, text='Load', width=6, command=lambda:
app.flow_show_layout(self.stage.get())).pack(side='left')
        ttk.Button(bar, text='Fit', width=5,
command=self.fit).pack(side='left', padx=4)
        ttk.Separator(bar, orient='vertical').pack(side='left',
fill='y', padx=6)
        for k, t in (('cells', 'Cells'), ('fill', 'Fill/tap'),
('pins', 'Pins'), ('route', 'Routing'), ('power', 'Power')):
            ttk.Checkbutton(bar, text=t, variable=self.show[k],
style='Toolbutton', command=self.redraw).pack(side='left', padx=1)
        self.layer_mb = ttk.Menubutton(bar, text='Layers');
self.layer_menu = tk.Menu(self.layer_mb, tearoff=0);
self.layer_mb['menu'] = self.layer_menu;
self.layer_mb.pack(side='left', padx=4)
        ttk.Button(bar, text='OpenROAD GUI…', command=lambda:
app.flow_open_gui(self.stage.get())).pack(side='left', padx=4)
        bar2 = ttk.Frame(self, padding=(8, 0)); bar2.pack(side='top', fill='x')
        self.info = ttk.Label(bar2, text='No layout loaded. Flow >
Open ORFS config…, run the stages, then Load.', style='Hint.TLabel');
self.info.pack(side='left')
        self.cv = tk.Canvas(self, highlightthickness=0=app.T['bg']); self.cv.pack(fill='both', expand=True)
        self.hint = ttk.Label(self, text='', style='Hint.TLabel');
self.hint.pack(side='bottom', fill='x', padx=8)
        c = self.cv
        c.bind('<Configure>', lambda e: self.later());
c.bind('<ButtonPress-1>', self.on_press); c.bind('<B1-Motion>',
self.on_drag); c.bind('<ButtonRelease-1>', self.on_release)
        c.bind('<ButtonPress-2>', self.on_press);
c.bind('<B2-Motion>', self.on_drag); c.bind('<Motion>', self.on_hover)
        c.bind('<Button-4>', lambda e: self.zoom(1.2, e.x, e.y));
c.bind('<Button-5>', lambda e: self.zoom(1 / 1.2, e.x, e.y));
c.bind('<MouseWheel>', lambda e: self.zoom(1.2 if e.delta > 0 else 1 /
1.2, e.x, e.y))
        for k, dx, dy in (('Left', -1, 0), ('Right', 1, 0), ('Up', 0,
1), ('Down', 0, -1)):
            c.bind('<%s>' % k, lambda e, dx=dx, dy=dy: self.step(dx,
dy)); c.bind('<Shift-%s>' % k, lambda e, dx=dx, dy=dy: self.step(dx *
5, dy * 5))
        c.bind('<Enter>', lambda e: c.focus_set())

    # ---- data ----
    def clear(self):
        self.D = None; self.cv.delete('all');
self.info.configure(text='No layout loaded.')
    def set_design(self, D, lef, label=''):
        self.D = D; self.lef = lef; self.hl = set(); u = D.dbu; lm =
lef.macros if lef else {}
        X0 = []; Y0 = []; X1 = []; Y1 = []; K = []; self.names = [];
self.types = []
        rx_seq =
re.compile(r'(?i)(dff|dfx|dfr|dfs|sdf|latch|dlx|dlat|_ff|ff_|sff)');
rx_buf = re.compile(r'(?i)(clkbuf|clkinv|clkdly|buf|inv)'); rx_fill =
re.compile(r'(?i)(fill|tap|decap|antenna|welltap|endcap|dummy)')
        wsum = 0
        for name, ty, x, y, ori, st in D.comps:
            mw, mh, cls = lm.get(ty, (0, 0, ''))
            w = int(mw * u); h = int(mh * u)
            if not w: w = h = int(1.0 * u)
            if ori in ('W', 'E', 'FW', 'FE'): w, h = h, w
            X0.append(x); Y0.append(y); X1.append(x + w); Y1.append(y
+ h); wsum += w
            if cls.startswith('BLOCK'): k = 3
            elif cls.startswith('CORE SPACER') or cls.startswith('CORE
WELLTAP') or cls.startswith('ENDCAP') or cls.startswith('CORE
ANTENNACELL') or rx_fill.search(ty): k = 4
            elif rx_seq.search(ty): k = 1
            elif rx_buf.search(ty): k = 2
            else: k = 0
            K.append(k); self.names.append(name); self.types.append(ty)
        self.X0, self.Y0, self.X1, self.Y1, self.K = X0, Y0, X1, Y1, K
        self.avgw = (wsum / len(X0)) if X0 else u; self.byname = None
        # density bins for the zoomed-out view
        dx0, dy0, dx1, dy1 = D.die; nb = 96; bw = max(1, (dx1 - dx0) /
nb); bh = max(1, (dy1 - dy0) / nb); self.bins = [0.0] * (nb * nb);
self.nb = nb; self.bw = bw; self.bh = bh
        for i in range(len(X0)):
            if K[i] == 4: continue
            bx = int(((X0[i] + X1[i]) / 2 - dx0) / bw); by =
int(((Y0[i] + Y1[i]) / 2 - dy0) / bh)
            if 0 <= bx < nb and 0 <= by < nb: self.bins[by * nb + bx]
+= (X1[i] - X0[i]) * (Y1[i] - Y0[i])
        # layers
        order = [n for n, t in (lef.layers if lef else []) if t ==
'ROUTING']; seen = []
        for (ly, sp) in D.segs:
            if ly not in seen: seen.append(ly)
        self.layers = [l for l in order if l in seen] + [l for l in
seen if l not in order]
        self.lcol = {l: LAYER_COLORS[i % len(LAYER_COLORS)] for i, l
in enumerate(self.layers)}
        self.layer_menu.delete(0, 'end')
        for l in self.layers:
            v = self.layer_on.setdefault(l,
tk.BooleanVar(value=True)); self.layer_menu.add_checkbutton(label=l,
variable=v, command=self.redraw, foreground=self.lcol[l])
        nseg = sum(len(a) // 4 for a in D.segs.values())
        self.info.configure(text='%s   %d cells · %d pins · %d nets ·
%d wire segments · die %.0f × %.0f µm' % (label, len(D.comps),
len(D.pins), D.nets, nseg, (dx1 - dx0) / u, (dy1 - dy0) / u))
        self.fit()
    def index(self):
        if self.byname is None: self.byname = {_unesc(n): i for i, n
in enumerate(self.names)}
        return self.byname

    # ---- view transform ----
    def fit(self):
        if not self.D: return
        W = m self.cv.winfo_width()); H = max(50,
self.cv.winfo_height()); x0, y0, x1, y1 = self.D.die
        self.z = min((W - 20) / max(1, x1 - x0), (H - 20) / max(1, y1
- y0)); self.vx = x0 - 10 / self.z; self.vy = y0 - 10 / self.z;
self.later()
    def zoom(self, f, sx=None, sy=None):
        if not self.D: return
        H = self.cv.winfo_height(); sx = sx if sx is not None else
self.cv.winfo_width() / 2; sy = sy if sy is not None else H / 2
        wx = self.vx + sx / self.z; wy = self.vy + (H - sy) / self.z
        self.z = max(1e-5, min(self.z * f, 500.0)); self.vx = wx - sx
/ self.z; self.vy = wy - (H - sy) / self.z; self.later()
    def step(self, dx, dy):
        self.vx += dx * 80 / self.z; self.vy += dy * 80 / self.z;
self.later(); return 'break'
    def later(self):
        if self.after_id is None: self.after_id = self.after(15, self._do)
    def _do(self): self.after_id = None; self.redraw()
    def w2s(self, x, y, H): return (x - self.vx) * self.z, H - (y -
self.vy) * self.z

    # ---- interaction ----
    def on_press(self, e): self.press = (e.x, e.y, self.vx, self.vy,
False, bool(e.state & 0x4)); self.cv.focus_set()
    def on_drag(self, e):
        if not self.press: return
        x, y, vx, vy, mv, ct = self.press
        if abs(e.x - x) + abs(e.y - y) > 3: mv = True
        self.press = (x, y, vx, vy, mv, ct)
        if mv: self.vx = vx - (e.x - x) / self.z; self.vy = vy + (e.y
- y) / self.z; self.later()
    def at(self, sx, sy):
        if not self.D: return None
        H = self.cv.winfo_height(); wx = self.vx + sx / self.z; wy =
self.vy + (H - sy) / self.z; best = None
        X0, Y0, X1, Y1, K = self.X0, self.Y0, self.X1, self.Y1,
self.K; show_fill = self.show['fill'].get()
        for i in range(len(X0) - 1, -1, -1):
            if X0[i] <= wx <= X1[i] and Y0[i] <= wy <= Y1[i] and
(show_fill or K[i] != 4): best = i; break
        return best
    def on_release(self, e):
        p = self.press; self.press = None
        if not p or p[4]: return
        i = self.at(e.x, e.y)
        if i is None:
            if not p[5]: self.app.layout_pick(None, False)
            return
        self.app.layout_pick(_unesc(self.names[i]), p[5])
    def on_hover(self, e):
        i = self.at(e.x, e.y)
        if i is None:
            H = self.cv.winfo_height(); u = self.D.dbu if self.D else 1
            if self.D: self.hint.configure(text='x %.2f  y %.2f µm
(drag: pan · wheel: zoom · click: select in schematic · Ctrl-click:
add)' % ((self.vx + e.x / self.z) / u, (self.vy + (H - e.y) / self.z)
/ u))
        elself.hint.configure(text='%s  (%s)  at %.2f, %.2f µm' %
(_unesc(self.names[i]), self.types[i], self.X0[i] / self.D.dbu,
self.Y0[i] / self.D.dbu))
    def set_highlight(self, names):
        if not self.D: return
        idx = self.index(); self.hl = {idx[n] for n in names if n in
idx}; self.later()
        if self.hl:
            i = next(iter(self.hl)); W = self.cv.winfo_width(); H =
self.cv.winfo_height(); cx = (self.X0[i] + self.X1[i]) / 2; cy =
(self.Y0[i] + self.Y1[i]) / 2
            if not (self.vx <cx < self.vx + W / self.z and self.vy <
cy < self.vy + H / self.z): self.center_on_highlight()
    def center_on_highlight(self):
        if not self.hl: return
        i = next(iter(self.hl)); H = self.cv.winfo_height(); W =
self.cv.winfo_width()
        self.vx = (self.X0[i] + self.X1[i]) / 2 - W / 2 / self.z;
self.vy = (self.Y0[i] + self.Y1[i]) / 2 - H / 2 / self.z; self.later()

    # ---- drawing ----
    def redraw(self):
        c = self.cv; c.delete('all')
        D = self.D
        if not D: return
        W = c.winfo_width(); H = c.winfo_height(); z = self.z; T =
self.app.T; vx, vy = self.vx, self.vy
        wx1 = vx + W / z; wy1 = vy + H / z
        x0, y0, x1, y1 = D.die; ax, ay = self.w2s(x0, y1, H); bx, by =
self.w2s(x1, y0, H)
        c.create_rectangle(ax, ay, bx, by, outline=T['ink'],
fill=T['fill'], width=1)
        N = len(self.X0); X0, Y0, X1, Y1, K = self.X0, self.Y0,
self.X1, self.Y1, self.K; fill_on = self.show['fill'].get()
        lod = False
        if self.show['cells'].get() and N:
            vis = [i for i in range(N) if X1[i] >= vx and X0[i] <= wx1
and Y1[i] >= vy and Y0[i] <= wy1 and (fill_on or K[i] != 4)]
            if self.avgw * z < 1.4 or len(vis) > 45000:
                lod = True
                nb = self.nb; dx0, dy0 = D.die[0], D.die[1]; cap =
self.bw * self.bh; mx = max(self.bins) or 1.0
                for by_ in range(nb):
                    for bx_ in range(nb):
                        a = self.bins[by_ * nb + bx_]
                        if a <= 0: continue
                        t = min(1.0, a / cap); r = int(70 + 170 * t);
g = int(120 + 60 * (1 - abs(t - 0.5) * 2)); b = int(230 - 190 * t)
                        sx0, sy0 = self.w2s(dx0 + bx_ * self.bw, dy0 +
by_ * self.bh, H); sx1, sy1 = self.w2s(dx0 + (bx_ + 1) * self.bw, dy0
+ (by_ + 1) * self.bh, H)
                        c.create_rectangle(sx0, sy0, sx1, sy1,
fill='#%02x%02x%02x' % (r, g, b), outline='')
            else:
                cols = self.CELL_COL; keys = ('comb', 'seq', 'buf',
'macro', 'fill'); ol = (self.avgw * z) > 6
                for i in vis:
                    sx0, sy0 = self.w2s(X0[i], Y1[i], H); sx1, sy1 =
self.w2s(X1[i], Y0[i], H)
                    c.create_rectangle(sx0, sy0, sx1, sy1,
fill=cols[keys[K[i]]], outline=T['ink'] if ol else '', width=1)
                if self.avgw * z > 28:
                    for i in vis[:1500]:
                        sx0, sy0 = self.w2s(X0[i], Y1[i], H); sx1, sy1
= self.w2s(X1[i], Y0[i], H)
                        if K[i] != 4: c.create_text((sx0 + sx1) / 2,
(sy0 + sy1) / 2, text=self.types[i].split('__')[-1],
font=(self.app.fam, 7), fill=T['ink'])
        if self.show['route'].get() or self.show['power'].get():
            tot = 0; todo = []
            for (ly, sp), arr in D.segs.items():
                if sp and not self.show['power'].get(): continue
                if not sp and not self.show['route'].get(): continue
                v = self.layer_on.get(ly)
                if v is not None and not v.get(): continue
                todo.append((ly, sp, arr)); tot += len(arr) // 4
            stride = max(1, tot // 60000 + (1 if tot > 60000 else 0)); drawn = 0
            for ly, sp, arr in todo:
                col = self.lcol.get(ly, '#888'); wd = 2 if (sp or z >
0.5) else 1
                for k in range(0, len(arr), 4 * stride):
                    a, b, cx, d = arr[k], arr[k + 1], arr[k + 2], arr[k + 3]
                    if max(a, cx) < vx or min(a, cx) > wx1 or max(b,
d) < vy or min(b, d) > wy1: continue
                    sa, sb = self.w2s(a, b, H); sc, sd = self.w2s(cx, d, H)
                    c.create_line(sa, sb, sc, sd, fill=col, width=wd);
drawn += 1
            if stride > 1: lod = True
        if self.show['pins'].get():
            for name, dr, x, y, ly in D.pins:
                if x < vx or x > wx1 or y < vy or y > wy1: continue
                sx, sy = self.w2s(x, y, H); col = '#2ca05a' if dr ==
'INPUT' else '#d8443b' if dr == 'OUTPUT' else '#c58a14'
                c.create_rectangle(sx - 3, sy - 3, sx + 3, sy + 3,
fill=col, outline='')
                if z * D.dbu > 20: c.create_text(sx + 5, sy,
text=name, anchor='w', font=(self.app.fam, 7), fill=T['ink'])
        for i in self.hl:
            sx0, sy0 = self.w2s(X0[i], Y1[i], H); sx1, sy1 =
self.w2s(X1[i], Y0[i], H); cx = (sx0 + sx1) / 2; cy = (sy0 + sy1) / 2
            c.create_rectangle(sx0 - 1, sy0 - 1, sx1 + 1, sy1 + 1,
outline=T['acc'], width=2)
            c.create_oval(cx - 9, cy - 9, cx + 9, cy + 9,
outline=T['acc'], width=2)
        if lod: c.create_text(8, H - 8, anchor='sw', text='overview
(zoom in for individual cells / all wires)', fill=T['ink'],
font=(self.app.fam, 8))


class FlowMixin:
    """Flow menu, Flow tab (config.mk, make stages, log) and the glue
to the schematic / layout"""
    def flow_init(self):
        self.proj = OrfsProject(); self.flow_q = []; self.flow_proc =
None; self.flow_cur = None; self.flow_auto = tk.BooleanVar(value=True)
        self.flow_log_lines = 0; self.flow_cb = None; self.lef = None
        try:
            with open(CFG_FILE) as f: self.flow_prefs = json.load(f)
        except Exception: self.flow_prefs = {}

    def flow_save_prefs(self):
        try:
            with open(CFG_FILE, 'w') as f: json.dump(self.flow_prefs, f)
        except OSError: pass

    def build_flow(self, nb):
        f = ttk.Frame(nb); nb.add(f, text='Flow'); self.flow_frame = f
        bar = ttk.Frame(f); bar.pack(fill='x', padx=6, pady=(4, 0))
        ttk.Button(bar, text='Open config.mk…',
command=self.flow_open_config).pack(side='left')
        ttk.Butt(bar, text='Edit…',
command=self.flow_edit_config).pack(side='left', padx=2)
        self.flow_btn = {}
        ttk.Separator(bar, orient='vertical').pack(side='left',
fill='y', padx=6)
        for st, title, tgt, n in ORFS_STAGES:
            b = ttk.Button(bar, text=title.split(' (')[0], width=11,
command=lambda s=st: self.flow_run([s])); b.pack(side='left', padx=1);
self.flow_btn[st] = b
        ttk.Button(bar, text='Run all', width=8, command=lambda:
self.flow_run([s[0] for s in ORFS_STAGES])).pack(si='left', padx=(6,
1))
        self.flow_stop_btn = ttk.Button(bar, text='Stop', width=6,
command=self.flow_stop, state='disabled');
self.flow_stop_btn.pack(side='left', padx=1)
        ttk.Checkbutton(bar, text='Load results into viewer',
variable=self.flow_auto).pack(side='right')
        self.flow_info = ttk.Label(f, text='No project. Open an ORFS
design config.mk (lines like  export DESIGN_NAME = gcd).',
style='Hint.TLabel'); self.flow_info.pack(fill='x', padx=8, pady=2)
        body = ttk.Frame(f); body.pack(fill='both', expand=True,
padx=6, pady=(0, 6))
        self.flow_text = tk.Text(body, height=6, wrap='none',
relief='flat', bg=self.T['fill'], fg=self.T['ink'], font=(self.mono,
9), state='disabled')
        ys = ttk.Scrollbar(body, orient='vertical',
command=self.flow_text.yview);
self.flow_text.configure(yscrollcommand=ys.set)
        self.flow_text.pack(side='left', fill='both', expand=True);
ys.pack(side='right', fill='y')
        self.flow_text.tag_configure('err', foreground='#d8443b');
self.flow_text.tag_configure('warn', foreground='#c58a14');
self.flow_text.tag_configure('hdr', font=(self.mono, 9, 'bold'))

    def build_flow_menu(self, m):
        fm = tk.Menu(m, tearoff=0); m.add_cascade(label='Flow', menu=fm)
        fm.add_command(label='Open ORFS config.mk…',
command=self.flow_open_config)
        fm.add_command(label='Edit config.mk…', command=self.flow_edit_config)
        fm.add_command(label='Set ORFS flow directory…',
command=self.flow_set_home)
        fm.add_command(label='Shoables…', command=self.flow_show_vars)
        fm.add_command(label='Check tools (yosys, openroad, make)',
command=self.flow_check_tools); fm.add_separator()
        for st, title, tgt, n in ORFS_STAGES:
fm.add_command(label='Run  %s' % title, command=lambda s=st:
self.flow_run([s]))
        fm.add_command(label='Run all stages', command=lambda:
self.flow_run([s[0] for s in ORFS_STAGES]))
        fm.add_command(label='Stop', command=self.flow_stop); fm.add_separator()
        fm.add_command(label='Load synesized netlist (+ Liberty)
into the viewer', command=lambda: self.flow_load_netlist())
        fm.add_command(label='Show layout of last stage',
command=lambda: self.flow_show_layout(None))
        fm.add_command(label='Clean results (make clean_all)',
command=lambda: self.flow_run(['clean']))
        fm.add_checkbutton(label='Load results into the viewer after
each stage', variable=self.flow_auto)

    # ---- logging ----
    def flow_log(self, s, tag=None):
        t = self.flow_text; t.configure(state='normal')
        if tag is None:
            if re.search(r'\[ERROR|^Error|\*\*\* |: error:|Traceback',
s): tag = 'err'
            elif re.search(r'\[WARNING|^Warning', s): tag = 'warn'
        t.insert('end', s + '\n', tag); self.flow_log_lines += 1
        if self.flow_log_lines > 20000: t.delete('1.0', '5000.0');
self.flow_log_lines -= 5000
        t.see('end'); t.configure(state='disabled')
    def flow_show_tab(self):
        try: self.bot.select(self.flow_frame)
        except Exception: pass

    # ---- project ----
    def flow_open_config(self, path=None):
        if not path:
            path = filedialog.askopenfilename(title='Open ORFS design
config.mk', initialdir=self.flow_prefs.get('last_dir') or None,
filetypes=[('ORFS config', '*.mk'), ('All files', '*')])
        if not path: return None
        try:
            items = self.proj.load(path, self.flow_prefs.get('flow_home', ''))
        except OSError as ex:
            messagebox.showerror(APP_NAME, 'Cannot read %s:\n%s' %
(path, ex)); return None
        self.flow_prefs['last_cfg'] = path;
self.flow_prefs['last_dir'] = os.path.dirname(path);
self.flow_save_prefs()
        self.lef = None; self.flow_update_info()
        self.flow_log('Project %s: %d variables from the design file'
% (path, len(items)), 'hdr')
        if not self.proj.flow_home: self.flow_log('Warning: the ORFS
flow directory (contains Makefile and scripts/) was not found. Use
Flow > Set ORFS flow directory…', 'warn')
        for need in ('DESIGN_NAME', 'PLATFORM'):
          if not self.proj.val(need): self.flow_log('Warning: %s is
not defined in the config' % need, 'warn')
        return items
    def flow_update_info(self):
        p = self.proj
        if not p.cfg: return
        done = [s[0] for s in ORFS_STAGES if p.stage_done(s[0])]
        self.flow_info.configure(text='%s   ·   design %s   ·
platform %s   ·   flow %s   ·   results: %s' %
(os.path.basename(p.cfg), p.val('DESIGN_NAME') or '?',
p.val('PLATFORM') or '?', p.flow_home or 'NOT FOUND', ', '.join(done)
orne yet'))
        for st, b in self.flow_btn.items():
b.configure(text=[s[1].split(' (')[0] for s in ORFS_STAGES if s[0] ==
st][0] + (' ✓' if st in done else ''))
    def flow_set_home(self):
        d = filedialog.askdirectory(title='ORFS flow directory (the
one with Makefile and scripts/)', initialdir=self.proj.flow_home or
None)
        if not d: return
        if not (os.path.isfile(os.path.join(d, 'Makefile')) and
os.path.isdir(os.path.join(d, 'scripts'))):
            if os.path.isfile(os.path.join( 'flow', 'Makefile')): d
= os.path.join(d, 'flow')
            else: messagebox.showwarning(APP_NAME, 'That directory has
no Makefile and scripts/ folder.'); return
        self.flow_prefs['flow_home'] = d; self.flow_save_prefs()
        if self.proj.cfg: self.flow_open_config(self.proj.cfg)
    def flow_edit_config(self):
        p = self.proj
        if not p.cfg: messagebox.showinfo(APP_NAME, 'Open a config.mk
first.'); return
        top = tk.Toplevel(self.root); top.title('Edit ' + p.cfg);
top.geometry('820x560'); top.transient(self.root)
        t = tk.Text(top, wrap='none', font=(self.mono, 10),
undo=True); t.pack(fill='both', expand=True)
        with open(p.cfg, encoding='utf-8', errors='replace') as f:
t.insert('1.0', f.read())
        bar = ttk.Frame(top, padding=6); bar.pack(fill='x')
        def save():
            with open(p.cfg, 'w', encoding='utf-8') as f:
f.write(t.get('1.0', 'end-1c'))
            self.flow_open_config(p.cfg); top.destroy()
        ttk.Button(bar, text='Save and reload',
command=save).pack(side='right'); ttk.Button(bar, text='Cancel',
command=top.destroy).pack(side='right', padx=6)
        ttk.Label(bar, text='Variables follow make syntax:  export
NAME = value   (?=, :=, += and $(VAR) work)',
style='Hint.TLabel').pack(side='left')
    def flow_show_vars(self):
        p = self.proj
        if not p.cfg: messagebox.showinfo(APP_NAME, 'Open a config.mk
first.'); return
        top = tk.Toplevel(self.root); top.title('Flow variables');
top.geometry('900x520'); top.transient(self.root)
        tv = ttk.Treeview(top, columns=('src', 'val'), show='tree
headings'); tv.heading('#0', text='Variable'); tv.heading('src',
text='From'); tv.heading('val', text='Expanded value')
        tv.column('#0', width=200); tv.column('src', width=80);
tv.column('val', width=580); tv.pack(fill='both', expand=True)
        for k in sorted(p.vars):
            if k in ('FLOW_HOME', 'DESIGN_HOME', 'WORK_HOME'): continue
            tv.insert('', 'end', text=k, values=('design' if k in
p.own else 'platform/default', p.val(k)))
    def flow_check_tools(self):
        fh = self.proj.flow_home; out = []
        for t in ('make', 'yosys', 'openroad'):
            path = find_tool(t, fh)
            ver = ''
            if path:
                try: ver = subprocess.run([path, '-version' if t !=
'make' else '--version'], capture_output=True, text=True,
timeout=15).stdout.strip().splitlines()[:1]; ver = ver[0] if ver else
''
                except Exception: pass
            out.append('%-9s %s' % (t, ('%s   %s' % (path, ver)) if
path else 'NOT FOUND (set %s_EXE or PATH; ORFS installs to
tools/install/)' % t.upper()))
        self.flow_log('\n'.join(out), 'hdr'); self.flow_show_tab(); return out

    # ---- running ----
    def flow_run(self, stages, then=None):
        p = self.proj
        if not p.cfg: messagebox.showinfo(APP_NAME, 'Open an ORFS
config.mk first (Flow menu).'); return False
        if not p.flow_home: messagebox.showwarning(APP_NAME, 'The ORFS
flow directory was not found.\n\nFlow > Set ORFS flow directory…');
return False
        if self.flow_proc is not None and self.flow_proc.alive():
messagebox.showinfo(APP_NAME, 'A flow stage is still running. Stop it
first.'); return False
        if not shutil.which('make'): messagebox.showerror(APP_NAME,
'`make` was not found on the PATH.'); return False
        self.flow_rc = None; self.flow_q = list(stages); self.flow_cb
= then; self.flow_show_tab(); self.flow_next(); return True
    def flow_next(self):
        if not self.flow_q:
            self.flow_sp_btn.configure(state='disabled');
self.flow_update_info()
            cb, self.flow_cb = self.flow_cb, None
            if cb: cb(0)
            return
        st = self.flow_q.pop(0); self.flow_cur = st
        tgt = 'clean_all' if st == 'clean' else [s[2] for s in
ORFS_STAGES if s[0] == st][0]
        cmd = self.proj.make_cmd(tgt, THREADS[0] if THREADS[0] else 0)
        self.flow_log('=== %s : %s' % (st, ' '.join(cmd)), 'hdr');
self.set_status('Flow: running %s …' % st)
        env = os.environ.copy()env['FLOW_HOME'] = self.proj.flow_home
        self.flow_proc = FlowRun(cmd, self.proj.flow_home, env);
self.flow_stop_btn.configure(state='normal'); self.root.after(80,
self.flow_poll)
    def flow_poll(self):
        fp = self.flow_proc
        if fp is None: return
        n = 0
        while n < 400:
            try: ln = fp.q.get_nowait()
            except queue.Empty: break
            n += 1
            if ln is None: self.flow_finished(); return
            self.flow_log(ln)
        self.set_status('Flow: %s running… %d s' % (self.flow_cur,
time.time() - fp.t0)); self.root.after(80, self.flow_poll)
    def flow_finished(self):
        fp = self.flow_proc; st = self.flow_cur; rc = fp.rc if fp.rc
is not None else -1; self.flow_proc = None; self.flow_rc = rc
        dt = time.time() - fp.t0
        if rc != 0:
            self.flow_log('=== %s FAILED (exit code %s, %.1f s)' %
(st, rc, dt), 'err'); self.flow_q = [];
self.flow_stop_btn.configure(state='disabled')
            self.set_status('Flow: %s faed (exit code %s). See the
Flow tab / logs in %s' % (st, rc, self.proj.logs_dir()));
self.flow_update_info()
            cb, self.flow_cb = self.flow_cb, None
            if cb: cb(rc)
            return
        self.flow_log('=== %s done (%.1f s)' % (st, dt), 'hdr');
self.flow_update_info()
        if st != 'clean' and self.flow_auto.get():
            if st == 'synth': self.flow_load_netlist(then=self.flow_next)
            else: self.flow_show_layout(st, then=self.flow_next)
            return
        self.flow_next()
    def flow_stop(self):
        self.flow_q = []
        if self.flow_proc is not None and self.flow_proc.alive():
            self.flow_proc.cancel(); self.flow_log('=== stop
requested', 'warn'); self.set_status('Flow: stopping…')
    def flow_wait(self, timeout=None):
        """pump the GUI until the running flow (and queued stages)
finished; used by the Tcl -wait option"""
        t0 = time.time()
        while self.flow_proc is not None or self.job or time.time() -
getattr(self, '_fb'0) < 3:
            self.root.update(); time.sleep(0.02)
            if timeout and time.time() - t0 > timeout: break

    # ---- results into the viewer ----
    def flow_load_netlist(self, then=None):
        p = self.proj; f = p.find_result('synth', ('v',))
        if not f: messagebox.showinfo(APP_NAME, 'No synthesis result
yet. Run the Synthesis stage first.'); (then and then()); return
        libs = [x for x in p.files('LIB_FILES') if os.path.isfile(x)]
        self.lef = None
        def load_nl(): self._fb = time.time();
self.load_netlist_files([f]); self.root.after(100, lambda:
self.flow_after_job(then))
        if self.D is not None:
            if self.D is not None and any(m.dirty for m in
self.D.modules.values()) and not messagebox.askyesno(APP_NAME, 'The
loaded netlist has unsaved ECO edits. Replace it with the new
synthesis result?', default='no'): (then and then()); return
            self.close_all(ask=False)
        self._fb = time.time()
        if libs and (self.lib is None):
            self.load_lib_files(libs, then=load_nl)
        else: load_nl()
    def flow_after_job(self, then):
        if self.job: self.root.after(100, lambda:
self.flow_after_job(then)); return
        if then: then()
    def flow_make_def(self, stage, cancel=None):
        """DEF of a stage: the flow's own .def, else convert the .odb
with OpenROAD"""
        p = self.proj; f = p.find_result(stage, ('def',))
        if f: return f
        odb = p.find_result(stage, ('odb',))
        if not odb: return None
        out = os.path.splitext(odb)[0] + '.ns.def'
        if os.path.exists(out) and os.path.getmtime(out) >=
os.path.getmtime(odb): return out
        exe = find_tool('openroad', p.flow_home)
        if not exe: raise RuntimeError('OpenROAD was not found (needed
to convert %s to DEF). Set OPENROAD_EXE or put it on the PATH.' %
os.path.basename(odb))
        tcl = os.path.join(tempfile.gettempdir(), 'ns_odb2def_%d.tcl'
% os.getpid())
        with open(tcl, 'w') as fh: fh.write('read_db {%s}\nwrite_def
{%s}\nexit\n' % (odb, out))
        r = subprocess.run([exe, '-no_init', '-exit', tcl],
capture_output=True, text=True, timeout=1800)
        if not os.path.exists(out): raise RuntimeError('OpenROAD could
not write the DEF:\n%s' % (r.stdout + r.stderr)[-1500:])
        return out
    def flow_show_layout(self, stage=None, then=None):
        p = self.proj
        if not p.cfg: messagebox.showinfo(APP_NAME, 'Open an ORFS
config.mk first.'); (then and then()); return
        if stage is None or stage == 'synth':
            stage = next((s[0] for s in reversed(ORFS_STAGES[1:]) if
p.stage_done(s[0])), None)
            if stage is None: messagebox.showinfo(APP_NAME, 'No placed
result yet. Run Floorplan or later.'); (then and then()); return
        lefs = [x for x in (p.files('TECH_LEF') + p.files('SC_LEF') +
p.files('ADDITIONAL_LEFS')) if os.path.isfile(x)]
        def work(prog, cancel):
            prog(0.02, 'locating %s result' % stage); f =
self.flow_make_def(stage)
            if not f: raise RuntimeError('No %s result (.def or .odb)
in %s' % (stage, p.results_dir()))
            lef = self.lef
            if lef is None: prog(0.05, 'reading %d LEF file(s)' %
len(lefs)); lef = parse_lef(lefs)
            prog(0.08, 'reading %s' % os.path.basename(f)); text =
read_text(f); D = parse_def(text, prog, cancel); del text
            return D, lef, f
        def done(res):
            D, lef, f = res; self.lef = lef; self.layout.set_design(D,
lef, '%s · %s' % (stage, os.path.basename(f)));
self.layout.stage.set(stage)
           self.cn.select(self.layout); self.sync_layout_sel();
self.set_status('Layout loaded: %s' % os.path.basename(f))
            if then: self.root.after(50, then)
        self._fb = time.time()
        self.run_job('Loading %s layout' % stage, work, done)
    def flow_open_gui(self, stage):
        p = self.proj; t = ORFS_GUI_TARGET.get(stage)
        if not p.cfg or not t: messagebox.showinfo(APP_NAME, 'Open a
config.mk and pick a placed stage first.'); return
        self.flow_log('=== launching the OpenROAD GUI: make %s' % t, 'hdr')
        try: subprocess.Popen(p.make_cmd(t), cwd=p.flow_home,
start_new_session=True, stdout=subprocess.DEVNULL,
stderr=subprocess.DEVNULL)
        except OSError as ex: messagebox.showerror(APP_NAME, 'Cannot
start make: %s' % ex)

    # ---- cross probing ----
    def layout_pick(self, name, additive):
        if not self.D:
            self.set_status('Cell %s (load the synthesized netlist to
cross-probe it in the schematic)' % name if name else ''); return
        if name is None: self.select_none(); return
        try: self.shell.run('select_inst', [name] + (['-add'] if
additive else []) + ['-zoom'])
        except Exception as ex: self.set_status('%s is not in the
loaded netlist (%s)' % (name, ex))
    def sync_layout_sel(self):
        lp = getattr(self, 'layout', None)
        if lp is None or not lp.D: return
        try: names = [found_str(f).lstrip('/') for f in
self.selection_found() if f.kind == 'inst']
        except Exception: names = []
        lp.set_highlight(names)


class App(FlowMixin):
    def __init__(self, root, args=None):
        self.root = root; self.flow_init(); self.D = None; self.lib =
None; self.lib_name = ''; self.lib_files = []; self.lib_notes = [];
self.lib_check = 'strict'; self.v_name = ''; self.save_path = None
        self.top = None; self.root_mod = None; self.mod = None;
self.path = []          # path: [(inst, module)]
        self.theme_name = 'light'; self.T = THEMES['light']
        self.vx = -40.0; self.vy = -40.0; self.z = 1.0
        self.sel_node = None; self.sel_sig = None; self.sel_cell =
None; self.sel_pin = None
        self.mode = tk.StringVar(value='select'); self.pending = None;
self.press = None; self.wire_drag = None; self.node_drag = None
        self.batch_depth = 0; self._need_after = False; self.undo =
[]; self.redo = []; self.issues = []; self.vl_issues = [];
self.lib_issues = []
        self.v_snap = tk.BooleanVar(value=True); self.v_grid =
tk.BooleanVar(value=True); self.v_block = tk.BooleanVar(value=False)
        self.v_pins = tk.BooleanVar(value=False); self.v_nets =
tk.BooleanVar(value=False); self.v_wires = tk.StringVar(value='auto')
        self.v_auto = tk.BooleanVar(value=True); self.eco_style =
tk.StringVar(value='plain')
        self.f_err = tk.BooleanVar(value=True); self.f_warn =
tk.BooleanVar(value=True); self.f_info = tk.BooleanVar(value=False)
        self.redraw_pending = False; self.chk_job = None;
self.need_fit = False; self.flags_cache = None; self.dnd = None
        self.last_export = None; self.hover_pin = None; self.pan =
None; self.vl_queue = queue.Queue()
        self.msel = None; self.hl_groups = []; self.hl_ver = 0;
self._hl_cache = None; self.found_list = []; self.found_idx = -1;
self.find_win = None; self.search_root = None; self.job = None;
self.placing = None
        self.setup_fonts(); self.build_ui(); self.apply_theme();
self.bind_keys()
        self.set_status('Open a Liberty file and a netlist, or load
the sample.')
        if args:
            if args.get('sample'): self.load_sample()
            libs = args.get('lib') or []; nets = args.get('netlist') or []
            libs = libs if isinstance(libs, list) else [libs]; nets =
nets if isinstance(nets, list) else [nets]
            if libs: self.load_lib_files(libs, then=(lambda:
self.load_netlist_files(nets)) if nets else None)
            elif nets: self.load_netlist_files(nets)
        self.root.after(100, self.poll_queue); self.root.after(600,
self.fit_view)

    # ---------------- fonts and theme ----------------
    def setup_fonts(self):
        self.fam = pick_font(['DejaVu Sans', 'Liberation Sans', 'Noto
Sans', 'Ubuntu', 'Cantarell', 'Arial', 'Helvetica'], 'TkDefaultFont')
        self.mono = pick_font(['DejaVu Sans Mono', 'Liberation Mono',
'Noto Sans Mono', 'Ubuntu Mono', 'Courier New'], 'TkFixedFont')
        for nm in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont',
'TkHeadingFont', 'TkCaptionFont'):
            try: tkfont.nametofont(nm).configure(family=self.fam, size=10)
            except tk.TclError: pass
        try: tkfont.nametofont('TkFixedFont').configure(family=self.mono,
size=10)
        except tk.TclError: pass
        tkfont.nametofont('TkHeadingFont').configure(weight='bold')

    def apply_theme(self):
        T = self.T; st = ttk.Style(self.root)
        try: st.theme_use('clam')
        except tk.TclError: pass
        p = T['panel']
        st.configure('.', background=p, foreground=T['ink'],
fieldbackground=T['fill'], bordercolor=T['grid_major'], lightcolor=p,
darkcolor=p, troughcolor=T['grid'], focuscolor=T['acc'])
        st.configure('TFrame', background=p); st.configure('TLabel',
background=p, foreground=T['ink'])
        st.configure('TButton', background=T['fill'],
foreground=T['ink'], padding=(8, 3), bordercolor=T['grid_major'])
        st.map('TButton', background=[('active', T['acc_soft'])],
bordercolor=[('active', T['acc'])])
        st.configure('Toolbutton', background=T['fill'],
foreground=T['ink'], padding=(8, 3))
        st.map('Toolbutton', background=[('selected', T['acc_soft']),
('active', T['acc_soft'])], foreground=[('selected', T['acc'])])
        st.configure('TCheckbutton', background=p,
foreground=T['ink']); st.configure('TRadiobutton', background=p,
foreground=T['ink'])
        st.configure('TEntry', fieldbackground=T['fill'],
foreground=T['ink']); st.configure('TCombobox',
fieldbackground=T['fill'], foreground=T['ink'], background=T['fill'])
        st.map('TCombobox', fieldbackground=[('readonly', T['fill'])],
foreground=[('readonly', T['ink'])], selectbackground=[('readonly',
T['fill'])], selectforeground=[('readonly', T['ink'])])
        st.configure('Treeview', background=T['fill'],
fieldbackground=T['fill'], foreground=T['ink'], rowheight=22,
bordercolor=T['grid_major'])
        st.map('Treeview', background=[('selected', T['acc_soft'])],
foreground=[('selected', T['ink'])])
        st.configure('Treeview.Heading', background=p,
foreground=T['mute'], relief='flat')
        st.configure('TNotebook', background=p,
bordercolor=T['grid_major']); st.configure('TNotebook.Tab',
background=p, foreground=T['mute'], padding=(12, 4))
        st.map('TNotebook.Tab', background=[('selected', T['fill'])],
foreground=[('selected', T['ink'])])
        st.configure('TPanedwindow', background=T['grid_major'])
        st.configure('Hint.TLabel', background=p, foreground=T['mute'])
        st.configure('Err.TLabel', background=p, foreground=T['bad'])
        self.root.configure(background=p)
        self.canvas.configure(bg=T['bg'])
        for w in (self.src_text, self.eco_text, self.vl_text):
w.configure(bg=T['fill'], fg=T['ink'], insertbackground=T['ink'],
selectbackground=T['acc_soft'])
        self.prob.tag_configure('error', foreground=T['bad']);
self.prob.tag_configure('warn', foreground=T['clock']);
self.prob.tag_configure('info', foreground=T['mute'])
        self.prev.configure(bg=T['bg'])
        self.request_redraw()

    def set_theme(self, name):
        self.theme_name = name; self.T = THEMES[name];
self.apply_theme(); self.refresh_inspector(); self.draw_preview()

    # ---------------- UI construction ----------------
    def build_ui(self):
        r = self.root; r.title(APP_NAME); r.geometry('1480x900');
r.minsize(980, 600)
        self.build_menu()
        tb = ttk.Frame(r, padding=(6, 4)); tb.pack(side='top', fill='x')
        def btn(txt, cmd, w=None):
            b = ttk.Button(tb, text=txt, command=cmd, width=w);
b.pack(side='left', padx=2); return b
        btn('Liberty…', self.open_lib, 9); btn('Netlist…',
self.open_netlist, 9); btn('Sample', self.load_sample, 8)
        ttk.Separator(tb, ot='vertical').pack(side='left', fill='y', padx=6)
        ttk.Radiobutton(tb, text='Select', value='select',
variable=self.mode, style='Toolbutton',
command=self.on_mode).pack(side='left', padx=2)
        ttk.Radiobutton(tb, text='ECO wire', value='wire',
variable=self.mode, style='Toolbutton',
command=self.on_mode).pack(side='left', padx=2)
        ttk.Radiobutton(tb, text='Pan', value='pan',
variable=self.mode, style='Toolbutton',
command=self.on_mode).pack(side='left', padx=2)
        ttk.Separator(tb, orient='vertical').pack(side='left', fill='y', padx=6)
        self.b_undo = btn('Undo', self.do_undo, 7); self.b_redo =
btn('Redo', self.do_redo, 7)
        ttk.Separator(tb, orient='vertical').pack(side='left', fill='y', padx=6)
        btn('Fit', self.fit_view, 6)
        ttk.Checkbutton(tb, text='Grid', variable=self.v_grid,
style='Toolbutton', command=self.request_redraw).pack(side='left',
padx=2)
        ttk.Checkbutton(tb, text='Snap', variable=self.v_snap,
style='Toolbutton').pack(side='left', padx=2)
        ttk.Checkbutton(tb, text='Block view', variable=self.v_block,
style='Toolbutton', command=self.on_view_change).pack(side='left',
padx=2)
        ttk.Separator(tb, orient='vertical').pack(side='left', fill='y', padx=6)
        btn('Checks', lambda: self.run_checks_now(True));
btn('Verilator', lambda: self.run_verilator(False))
        ttk.Button(tb, text='Save ECO netlist',
command=self.save_netlist, width=16).pack(side='right', padx=2)
        # status bar
        sb = ttk.Frame(r, padding=(8, 2)); sb.pack(side='bottom', fill='x')
        self.status = ttk.Label(sb, text='', style='Hint.TLabel');
self.status.pack(side='left')
        self.coords = ttk.Label(sb, text='', style='Hint.TLabel');
self.coords.pack(side='right')
        self.sysmon = ttk.Label(sb, text='', style='Hint.TLabel',
width=34); self.sysmon.pack(side='right', padx=10)
        self._cpu_last = (time.time(), sum(os.times()[:2]));
self.root.after(1200, self.tick_sysmon)
        # main panes
        self.pw = ttk.PanedWindow(r, orient='horizontal');
self.pw.pack(fill='both', expand=True)
        left = ttk.Frame(self.pw); center = ttk.PanedWindow(self.pw,
orient='vertical'); right = ttk.Frame(self.pw, width=340)
        self.pw.add(left, weight=0); self.pw.add(center, weight=1);
self.pw.add(right, weight=0)
        self.build_left(left)
        cn = ttk.Notebook(center); center.add(cn, weight=1); self.cn = cn
        cf = ttk.Frame(cn); cn.add(cf, text='Schematic')
        self.layout = LayoutPane(cn, self); cn.add(self.layout, text='Layout')
        self.crumbs = ttk.Frame(cf, padding=(8, 3));
self.crumbs.pack(side='top', fill='x')
        self.canvas = tk.Canvas(cf, highlightthickness=0,
bg=self.T['bg'], cursor='arrow'); self.canvas.pack(fill='both',
expand=True)
        bot = ttk.Notebook(center); center.add(bot, weight=0); self.bot = bot
        self.build_bottom(bot); self.build_console(bot); self.build_flow(bot)
        self.insp = ttk.Frame(right, padding=8);
self.insp.pack(fill='both', expand=True)
        self.root.update_idletasks()
        try: self.pw.sashpos(0, 310); self.pw.sashpos(1, 1480 - 350)
        except tk.TclError: pass
        c = self.canvas
        c.bind('<Configure>', lambda e: self.request_redraw())
        c.bind('<ButtonPress-1>', self.on_press);
c.bind('<B1-Motion>', self.on_drag); c.bind('<ButtonRelease-1>',
self.on_release)
        c.bind('<Double-Button-1>', self.on_double)
        c.bind('<ButtonPress-2>', self.pan_start);
c.bind('<B2-Motion>', self.pan_move); c.bind('<ButtonRelease-2>',
lambda e: setattr(self, 'pan', None))
        c.bind('<ButtonPress-3>', self.on_context); c.bind('<Motion>',
self.on_hover)
        c.bind('<Button-4>', lambda e: self.zoom_at(1.15, e.x, e.y));
c.bind('<Button-5>', lambda e: self.zoom_at(1 / 1.15, e.x, e.y))
        c.bind('<MouseWheel>', lambda e: self.zoom_at(1.15 if e.delta
> 0 else 1 / 1.15, e.x, e.y))

    def build_menu(self):
        r = self.root; m = tk.Menu(r); r.config(menu=m)
        f = tk.Menu(m, tearoff=0); m.add_cascade(label='File', menu=f)
        f.add_command(label='Open Liberty…  (adds to the libraries
already read)', accelerator='Ctrl+L', command=self.open_lib)
        f.add_command(label='Close design and libraries',
command=self.close_all)
        f.add_command(label='Open netlist…', accelerator='Ctrl+O',
command=self.open_netlist)
        f.add_command(label='Load sample', command=self.load_sample);
f.add_separator()
        f.add_command(label='Save ECO netlist', accelerator='Ctrl+S',
command=self.save_netlist)
        f.add_command(label='Save Eetlist as…', command=lambda:
self.save_netlist(True))
        f.add_command(label='Export Verilator bundle (folder)…',
command=self.export_bundle)
        f.add_command(label='Export schematic as SVG…',
command=self.export_svg); f.add_separator()
        f.add_command(label='Quit', accelerator='Ctrl+Q', command=self.exit_now)
        e = tk.Menu(m, tearoff=0); m.add_cascade(label='Edit', menu=e)
        e.add_command(label='Undo', accelerator='Ctrl+Z',
command=self.do_undo); e.add_command(label='Redo'lerator='Ctrl+Shift+Z', command=self.do_redo)
        e.add_separator(); e.add_command(label='Clear selection and
highlights', command=self.clear_highlights)
        e.add_command(label='Delete selection', accelerator='Del',
command=self.delete_selection)
        e.add_command(label='Cancel pending ECO connection',
accelerator='Esc', command=self.cancel_pending)
        fm = tk.Menu(m, tearoff=0); m.add_cascade(label='Find', menu=fm)
        fm.add_command(label='Find objects by regexp…',
accelerator='CtrF', command=lambda: self.open_find('inst'));
fm.add_separator()
        for k, t in (('inst', 'Hierarchical instances by regexp…'),
('pin', 'Hierarchical pins by regexp…'), ('net', 'Hierarchical nets by
regexp…'),
                     ('port', 'Hierarchical module ports by regexp…'),
('topport', 'Top-module I/O ports by regexp…')):
            fm.add_command(label=t, command=lambda k=k: self.open_find(k))
        fm.add_separator(); fm.add_command(label='Zoom to selection',
accelerator='Z', commanm_to_selection)
        fm.add_command(label='Next match', accelerator='F3',
command=lambda: self.next_match(1)); fm.add_command(label='Previous
match', accelerator='Shift+F3', command=lambda: self.next_match(-1))
        fm.add_separator(); fm.add_command(label='Clear selection and
highlights', command=self.clear_highlights)
        v = tk.Menu(m, tearoff=0); m.add_cascade(label='View', menu=v)
        v.add_command(label='Fit to window', accelerator='F',
command=self.fit_view)
        v.add_checkbutton(label='Grid', variable=self.v_grid,
command=self.request_redraw); v.add_checkbutton(label='Snap to grid',
variable=self.v_snap)
        v.add_checkbutton(label='Block boxes instead of gate symbols',
variable=self.v_block, command=self.on_view_change)
        v.add_checkbutton(label='Pin names on gates',
variable=self.v_pins, command=self.request_redraw)
        v.add_checkbutton(label='Net names on wires',
variable=self.v_nets, command=self.request_redraw)
        w = tk.Menu(v, tearoff=0); v.add_cascade(label='Wires', menu=w)
        for lab, val in (('Auto', 'auto'), ('Routed', 'routed'), ('Net
labels', 'labels')): w.add_radiobutton(label=lab, value=val,
variable=self.v_wires, command=self.on_view_change)
        t = tk.Menu(v, tearoff=0); v.add_cascade(label='Theme', menu=t)
        t.add_command(label='Light', command=lambda:
self.set_theme('light')); t.add_command(label='Dark', command=lambda:
self.set_theme('dark'))
        v.add_command(label='Re-run auto layout', command=self.relayout)
        self.build_flow_menu(m)
        tl = tk.Menu(m, tearoff=0); m.add_cascade(label='Tools', menu=tl)
        tl.add_command(label='Run built-in checks', command=lambda:
self.run_checks_now(True))
        tl.add_command(label='Run Verilator lint', command=lambda:
self.run_verilator(False))
        tl.add_command(label='Run Verilator smoke simulation',
command=lambda: self.run_verilator(True))
        tl.add_command(label='Import Verilator log…',
command=self.import_vl_log); tl.add_separator()
        tl.add_command(lal='ECO console (Tcl / Python)',
accelerator='Ctrl+K', command=self.focus_console)
        tl.add_command(label='Source Tcl / Python command file…',
command=self.con_source); tl.add_separator()
        tl.add_checkbutton(label='Re-run checks after every edit',
variable=self.v_auto)
        self.v_libchk = tk.StringVar(value='strict'); lc = tk.Menu(tl,
tearoff=0); tl.add_cascade(label='Library check when loading a
netlist', menu=lc)
        for val, lab in (('strict', 'strict: abort if any cell is
missing fm the libraries'), ('warn', 'warn: load anyway, list the
missing cells'), ('off', 'off: no check')):
            lc.add_radiobutton(label=lab, variable=self.v_libchk,
value=val, command=lambda: setattr(self, 'lib_check',
self.v_libchk.get()))
        es = tk.Menu(tl, tearoff=0); tl.add_cascade(label='ECO net
name style', menu=es)
        es.add_radiobutton(label='Plain:
u_core_u_alu_U1_ZN_eco_net_1', value='plain', variable=self.eco_style)
        es.add_radiobutton(label='Escaped path:
\\/u_core/u_alu/U1/ZN_eco_net_1', value='escaped',
variable=self.eco_style)
        h = tk.Menu(m, tearoff=0); m.add_cascade(label='Help',
menu=h); h.add_command(label='Shortcuts and ECO workflow',
command=self.show_help)

    def build_left(self, left):
        nb = ttk.Notebook(left); nb.pack(fill='both', expand=True);
self.left_nb = nb
        hf = ttk.Frame(nb, padding=4); nb.add(hf, text='Hierarchy')
        hb = ttk.Frame(hf); hb.pack(side='top', fill='x')
        ttk.Button(hb, text='Expand all', width=11, command=lambda:
self.hier_expand_all()).pack(side='left', padx=(0, 4)); ttk.Button(hb,
text='Collapse all', width=11, command=lambda:
self.hier_collapse_all()).pack(side='left')
        hw = ttk.Frame(hf); hw.pack(side='top', fill='both', expand=True)
        self.hier = ttk.Treeview(hw, columns=('cells',), show='tree
headings', selectmode='browse'); self.hier.heading('#0', text='Module
(instance)'); self.hier.heading('cells', text='Cells')
        self.hier.column('#0', width=190, minwidth=600,
stretch=False); self.hier.column('cells', width=60, anchor='e',
stretch=False)
        ys = ttk.Scrollbar(hw, orient='vertical',
command=self.hier.yview); xs = ttk.Scrollbar(hw, orient='horizontal',
command=self.hier.xview)
        self.hier.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        xs.pack(side='bottom', fill='x'); ys.pack(side='right',
fill='y'); self.hier.pack(side='left', fill='both', expand=True)
        self.hier.bind('<<TreeviewSelect>>', self.on_hier_select);
self.hier_map = {}; self.hier_lock = False
        self.hier.bind('<<TreeviewOpen>>', self.on_hier_open)
        lf = ttk.Frame(nb, padding=4); nb.add(lf, text='Library')
        self.lib_info = ttk.Label(lf, text='No Liberty file loaded.',
style='Hint.TLabel', wraplength=270, justify='left');
self.lib_info.pack(fill='x')
        row = ttk.Frame(lf); row.pack(fill='x', pady=4)
        self.lib_q = tk.StringVar(); e = ttk.Entry(row,
textvariable=self.lib_q); e.pack(side='left', fill='x', expand=True)
        self.lib_q.trace_add('write', lambda *a: self.populate_library())
        self.lib_cat = tk.StringVar(value='All kinds'); cb =
ttk.Combobox(row, textvariable=self.lib_cat, state='readonly',
width=11, values=['All kinds', 'Combinational', 'Sequential',
'Macros', 'Modules'])
        cb.pack(side='left', padx=(4, 0));
cb.bind('<<ComboboxSelected>>', lambda e: self.populate_library())
        ttk.Label(lf, text='Drag a cell onto the sheet, or
double-click it.', style='Hint.TLabel').pack(fill='x')
        lw = ttk.Frame(lf); lw.pack(fill='both', expand=True, pady=4)
        self.libtree = ttk.Treeview(lw, columns=('desc', 'model'),
show='tree headings', selectmode='browse')
        self.libtree.heading('#0', text='Cell');
self.libtree.heading('desc', text='Function');
self.libtree.heading('model', text='Timing')
        self.libtree.column('#0', width=120);
self.libtree.column('desc', width=120); self.libtree.column('model',
width=48)
        ys2 = ttk.Scrollbar(lw, orient='vertical',
command=self.libtree.yview);
self.libtree.configure(yscrollcommand=ys2.set)
        self.libtree.pack(side='left', fill='both', expand=True);
ys2.pack(side='right', fill='y')
        self.libtree.bind('<<TreeviewSelect>>', self.on_lib_select)
        self.libtree.bind('<ButtonPress-1>', self.dnd_press, add='+');
self.libtree.bind('<B1-Motion>', self.dnd_motion, add='+')
        self.libtree.bind('<ButtonRelease-1>', self.dnd_release,
add='+'); self.libtree.bind('<Double-Button-1>', self.lib_double)
        self.prev = tk.Canvas(lf, height=120, highlightthickness=1,
highlightbackground='#ccc'); self.prev.pack(fill='x')

    # ================= ECO console (Tcl / Python) =================
    def build_console(self, nb):
        f = ttk.Frame(nb); nb.add(f, text='Console'); self.con_frame = f
        bar = ttk.Frame(f); bar.pack(fill='x', padx=6, pady=(4, 0))
        self.con_mode = tk.StringVar(value='tcl')
        ttk.Radiobutton(bar, text='Tcl', variable=self.con_mode,
value='tcl', command=self.con_mode_changed).pack(side='left')
        ttk.Radiobutton(bar, text='Python', variable=self.con_mode,
value='py', command=self.con_mode_changed).pack(side='left', padx=(6,
12))
        ttk.Button(bar, text='Source file…',
command=self.con_source).pack(side='left', padx=2)
        ttk.Button(bar, text='Help', command=lambda:
self.con_run('help')).pack(side='left', padx=2)
        ttk.Button(bar, text='Clear', command=lambda:
(self.con_text.configure(state='normal'), self.con_text.delete('1.0',
'end'), self.con_text.configure(state='disabled'))).pack(side='left',
padx=2)
        ttk.Label(bar, text='Up/Down: history   Tab: complete
<command> -help: usage', style='Hint.TLabel').pack(side='right')
        row = ttk.Frame(f); row.pack(side='bottom', fill='x', padx=6,
pady=(0, 6))
        body = ttk.Frame(f); body.pack(fill='both', expand=True, padx=6, pady=4)
        self.con_text = tk.Text(body, height=4, wrap='word',
relief='flat', bg=self.T['fill'], fg=self.T['ink'], font=(self.mono,
10), highlightthickness=1, highlightbackground=self.T['grid_major'],
state='disabled')
        sb = ttk.Scrollbar(body, command=self.con_text.yview);
self.con_text.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y');
self.con_text.pack(side='left', fill='both', expand=True)
        self.con_text.tag_configure('cmd', foreground=self.T['acc']);
self.con_text.tag_configure('err', foreground='#c0392b')
        self.con_prompt = ttk.Label(row, text='tcl>', font=(self.mono,
10, 'bold'), width=6); self.con_prompt.pack(side='left')
        self.con_var = tk.StringVar(); e = ttk.Entry(row,
textvariable=self.con_var, font=(self.mono, 10)); e.pack(side='left',
fill='x', expand=True)
        self.con_entry = e; self.con_hist = []; self.con_hi = 0
        e.bind('<Return>', lambda ev: self.con_enter());
e.bind('<Up>', lambda ev: self.con_history(-1)); e.bind('<Down>',
lambda ev: self.con_history(1)); e.bind('<Tab>', self.con_complete)
        self.shell = EcoShell(self); self.shell.out = self.con_write;
self.shell.install_tcl()
        self.con_write('ECO console. Tcl commands (get_cells,
all_fanout, eco_add_repeater, size_cell, ...) work on the loaded
design;\ntype "help" for the list, "<command> -help" for usage. Switch
to Python for the same commands as functions.')
        self.root.bind('<Control-k>', lambda ev: self.focus_console())
    def focus_console(self):
        self.bot.select(self.con_frame); self.con_entry.focus_set()
    def con_mode_changed(self):
        self.con_prompt.configure(text='tcl>' if self.con_mode.get()
== 'tcl' else 'py>'); self.con_entry.focus_set()
    def con_write(self, s, nl=True):
        t = self.con_text; t.configure(state='normal')
        tag = 'err' if s.startswith('Error') else None
        t.insert('end', s + ('\n' if nl else ''), tag); t.see('end');
t.configure(state='disabled')
        try: self.root.update_idletasks()
        except tk.TclError: pass
    def con_enter(self):
        line = self.con_var.get().strip(); self.con_var.set('')
        if not line: return
        self.con_hist.append(line); self.con_hi = len(self.con_hist)
        self.con_run(line)
    def con_run(self, line):
        self.con_text.configure(state='normal');
self.con_text.insert('end', '%s %s\n' % (self.con_prompt.cget('text'),
line), 'cmd'); self.con_text.configure(state='disabled')
        if not self.D and not
re.match(r'^\s*(help|source|set|puts|expr|proc)\b', line):
self.con_write('Error: load a netlist first.'); return
        (self.shell.exec_tcl if self.con_mode.get() == 'tcl' else
self.shell.exec_py)(line)
    def con_history(self, d):
        if not self.con_hist: return 'break'
        self.con_hi = max(0, min(len(self.con_hist), self.con_hi + d))
        self.con_var.set(self.con_hist[self.con_hi] if self.con_hi <
len(self.con_hist) else ''); self.con_entry.icursor('end'); return
'break'
    def con_complete(self, ev):
        s = self.con_var.get(); mm = re.search(r'([A-Za-z_]\w*)$', s)
        if mm:
            hits = sorted(n for n in self.shell.cmds if
n.startswith(mm.group(1)))
            if len(hits) == 1: self.con_var.set(s[:mm.start()] +
hits[0] + ' '); self.con_entry.icursor('end')
            elif hits: self.con_write('  '.join(hits))
        return 'break'
    def con_source(self):
        p = filedialog.askopenfilename(title='Source a Tcl or Python
command file', filetypes=[('Tcl / Python', '*.tcl *.py'), ('All
files', '*')])
        if not p: return
        self.focus_console(); self.con_write('source %s' % p)
        try:
            if p.lower().endswith('.py'): self.shell.source_file(p, True)
            else: self.shell.exec_tcl('source {%s}' % p)
        except Exception as ex: self.con_write('Error: %s' % ex)
        self.con_write('done.')

    # ================= ECO from the GUI (sizing, VT swap, repeaters,
clone) =================
    def gui_cmd(self, name, argv, quiet_ok=False):
        """run an ECO command from a menu; errors become dialogs"""
        try:
            r = self.shell.run(name, argv); self.refresh_eco_log(); return r
        except (CmdError, EcoError) as ex:
            msg = str(ex)
            if msg.startswith('non-eq-footprint'):
messagebox.showwarning('Non-equivalent footprint', msg,
parent=self.root)
            else: messagebox.showwarning(APP_NAME, msg, parent=self.root)
        except Exception as ex:
            log_crash(traceback.format_exc());
messagebox.showerror(APP_NAME, 'ECO command failed: %s' % ex,
parent=self.root)
        return None
    def hier_inst_names(self, nd):
        cur = self.cur_path(); names = ['/'.join(cur + (nd.ref.name,))]
        if self.msel and any(f.kind == 'inst' and f.path == cur and
f.name == nd.ref.name for f in self.msel):
            names = ['/'.join(f.path + (f.name,)) for f in self.msel
if f.kind == 'inst']
        return names
    def eco_menu_for_inst(self, menu, nd):
        lib = self.lib; cell = lib.cells.get(nd.ref.type) if lib else None
        names = self.hier_inst_names(nd); multi = len(names) > 1
        sub = tk.Menu(menu, tearoff=0); menu.add_cascade(label='ECO:
size / swap%s' % (' (%d selected cells)' % len(names) if multi else
''), menu=sub)
        if cell is None:
            sub.add_command(label='(not a library cell: no sizing)',
state='disabled')
        else:
            eq = [c for c in equiv_cells(lib, cell) if cell_vt(c) ==
cell_vt(cell)]
            if cell not in eq: eq.append(cell)
            i = eq.index(cell)
            up = eq[i + 1].name if i + 1 < len(eq) else None; dn =
eq[i - 1].name if i > 0 else None
            sub.add_command(label='Upsize  → %s' % up if up else
'Upsize (already largest)', state='normal' if up else 'disabled',
command=lamb: self.gui_cmd('eco_size', ['-inst', names, '-up']))
            sub.add_command(label='Downsize  → %s' % dn if dn else
'Downsize (already smallest)', state='normal' if dn else 'disabled',
command=lambda: self.gui_cmd('eco_size', ['-inst', names, '-down']))
            var = vt_variants(lib, cell); cv = cell_vt(cell)
            vm = tk.Menu(sub, tearoff=0); sub.add_cascade(label='Swap
VT  (now %s)' % (cv or 'unknown'), menu=vm, state='normal' if cv else
'disabled')
            for v in VT_ORDER:
              if v in var:
                    ok, why, der = footprint_check(cell, var[v])
                    vm.add_command(label='%s  → %s%s' % (v,
var[v].name, '' if ok else '   (non-eq footprint)'), state='disabled'
if v == cv else 'normal',
                                   command=lambda v=v:
self.gui_cmd('eco_swap_vt', ['-inst', names, '-to', v]))
            if not var: vm.add_command(label='(no VT variants in the
library)', state='disabled')
            sub.add_separator()
            sub.add_command(bel='Swap with equivalent cell…',
command=lambda: self.swap_cell_dialog(names, cell, True))
            sub.add_command(label='Swap with any library cell…',
command=lambda: self.swap_cell_dialog(names, cell, False))
        sub.add_separator()
        sub.add_command(label='Clone instance', command=lambda:
self.gui_cmd('clone_inst', ['-inst', names, '-num_clones', '1']))
        sub.add_command(label='Clone by count…', command=lambda:
self.clone_dialog(names))
        sub.add_separator()
        for self.out_nets_of(nd)[:4]:
            sub.add_command(label='eco_add_repeater on output net %s…'
% sg, command=lambda sg=sg: self.repeater_dialog(sg))
            sub.add_command(label='eco_delete_repeater after output
net %s' % sg, command=lambda sg=sg:
self.gui_cmd('eco_delete_repeater', ['-net', '/'.join(self.cur_path()
+ (sg,))]))
    def swap_cell_dialog(self, names, cell, equiv_only):
        lib = self.lib; cands = equiv_cells(lib, cell) if equiv_only
else [lib.cells[n] for n in lib.order]
        t = tk.Toplevel(self.root); top.title('Swap cell: %s' %
cell.name); top.transient(self.root)
        f = ttk.Frame(top, padding=10); f.pack(fill='both', expand=True)
        ttk.Label(f, text='%d instance%s of %s. Pick the new library
cell (footprint equivalence is checked before the swap).' %
(len(names), '' if len(names) == 1 else 's', cell.name),
wraplength=520).pack(anchor='w')
        q = tk.StringVar(); e = ttk.Entry(f, textvariable=q);
e.pack(fill='x', pady=4)
        tv = ttk.Treeview(f, columns=('vt', 'area', 'fp'), show='tree
headings', height=12, selectmode='browse')
        tv.heading('#0', text='Cell'); tv.heading('vt', text='VT');
tv.heading('area', text='Area'); tv.heading('fp', text='Footprint')
        tv.column('#0', width=220); tv.column('vt', width=60);
tv.column('area', width=70); tv.column('fp', width=170);
tv.pack(fill='both', expand=True)
        def fill(*_):
            tv.delete(*tv.get_children()); s = q.get().lower(); n = 0
            for c in cands:
                if s and s not in c.name.lower(): continue
                ok, why, der = footprint_check(cell, c)
                tv.insert('', 'end', iid=c.name, text=c.name + ('
(current)' if c is cell else ''), values=(cell_vt(c) or '-', c.area,
'equivalent' if ok else 'NON-EQ-FOOTPRINT')); n += 1
                if n >= 2000: break
        q.trace_add('write', fill); fill()
        def go():
            sel = tv.selection()
            if not sel: return
            top.destroy(); self.gui_cmd('size_cell', [names, sel[0]])
        b = ttk.Frame(f); b.pack(fill='x', pady=(8, 0)); ttk.Button(b,
text='Cancel', command=top.destroy).pack(side='right', padx=(6, 0));
ttk.Button(b, text='Swap', command=go).pack(side='right')
        tv.bind('<Double-Button-1>', lambda ev: go()); e.focus_set();
top.grab_set()
    def clone_dialog(self, names):
        r = FormDialog(self.root, 'Clone instances', [('n', 'Number of
clones per instance (%d instance%s selected)' % (len(names), '' if
len(names) == 1 else 's'), '2', None),
                        ('tie', 'Tie unconnected inputs of the clones
to 0', 'No', ['No', 'Yes'])], 'Clone').result
        if not r: return
        try: n = int(r['n'])
        except ValueError: n = 0
        if n < 1: messagebox.showwarning(APP_NAME, 'The clone count
must be a positive number.', parent=self.root); return
        if n * len(names) > 5000 and not messagebox.askyesno(APP_NAME,
'Create %d new instances?' % (n * len(names)), parent=self.root):
return
        args = ['-inst', names, '-num_clones', n] + (['-auto_tie',
'1'] if r['tie'] == 'Yes' else [])
        res = self.gui_cmd('clone_inst', args)
        if res: self.set_status('Created %d clone(s).' % len(res))
    def out_nets_of(self, nd):
        out = []
        for pin, ids in nd.pin_sigs.items():
            if nd.sym.pins.get(pin, {}).get('dir') == 'out':
                for sg in ids:
                    v = self.get_view()
                    if sg in v.sigs and not v.sigs[sg].stub and sg not
in out: out.append(sg)
        return out
    def net_prompt(self, title, default=''):
        r = FormDialog(self.root, title, [('net', 'Net (hierarchical
name, wildcards allowed for removal)', default, None)], 'OK').result
        return (r or {}).get('net', '').strip() or None
    def repeater_net_dialog(self):
        n = self.net_prompt('Insert repeater chain: which net?',
self.sel_sig or '')
        if not n: return
        cur = self.cur_path()
        if '/' not in n and cur and self.sel_sig == n: self.repeater_dialog(n)
        elif '/' not in n and not cur: self.repeater_dialog(n)
        else:
            # hierarchical name given: split off the module path
            self.repeater_dialog(n, absolute=True)
    def repeater_cells(self):
        out = []
        if self.lib:
            for n in self.lib.order:
                info = repeater_info(self.lib, n)
                if info: out.append('%s   (%s)' % (n, 'inverter' if
info[2] else 'buffer'))
        return out
    def repeater_sinks(self, hn, load_pin=None, sel=None):
        """hierarchical names of the selected sink pins that really
are loads of net <hn> (the clicked pin first)"""
        sh = self.shell; cands = []
        if load_pin: cands.append('/'.join(self.cur_path() + tuple(load_pin)))
        for f in (sel if sel is not None else self.selection_found()):
            if f.kind == 'pin' and f.sub: cands.append(found_str(f).lstrip('/'))
        try: path, m, base, key = sh.resolve_net(hn); leaves =
{(tuple(l[0]), l[2].name, l[3]) for l in sh.trace(path, key)[0] if
l[5] == 'input'}
        except Exception: return []
        out = []
        for c in cands:
            try: r = sh.resolve_pin(c)
            except Exception: continue
            if r['inst'] is not None and (tuple(r['path']),
r['inst'].name, r['pin']) in leaves and c not in out: out.append(c)
        return out

    def repeater_dialog(self, net_sig, load_pin=None, absolute=False, sel=None):
        """net_sig: signal id in the current module; load_pin: (inst,
pin) when started from a load pin.
        Sink pins that are selected in the schematic (or the clicked
pin) default to 'only these loads'."""
        cells = self.repeater_cells()
        if not cells: messagebox.showinfo(APP_NAME, 'The loaded
Liberty file has no single-input buffer or inverter cells.',
parent=self.root); return
        bufs = [c for c in cells if 'buffer' in c]
        hn = net_sig if absolute else '/'.join(self.cur_path() + (net_sig,))
        try:                                                    # a
pin on one bit of a bus: use that bit as the net
            path, m, base, key = self.shell.resolve_net(hn)
            if m.nets[base].vec and key == base and load_pin:
                ks =
self.shell.pin_nets(self.shell.resolve_pin('/'.join(self.cur_path() +
tuple(load_pin))))
                if len(ks) == 1: hn = self.shell.hname(ks[0][0], ks[0][1])
        except Exception: pass
        sinks = self.repeater_sinks(hn, load_pin, sel)
        only = 'Only selected sink pin%s (%s)' % ('s' if len(sinks) >
1 else '', ', '.join(sinks[:3]) + (' ...' if len(sinks) > 3 else ''))
if sinks else None
        fields = [('cell', 'Repeater cell (library list)', (bufs or
cells)[0], cells), ('stages', 'Stages (buffers, or inverter PAIRS)',
'2', None), ('prefix', 'Name prefix', 'eco_rpt', None)]
        if only: fields.append(('loads', 'Loads to move behind the
chain', only, [only, 'All loads of the net']))
        r = FormDialog(self.root, 'Insert repeater chain on %s' % hn,
fields, 'Insert').result
        if not r: return
        try: stages = int(r['stages'])
        except ValueError: messagebox.showwarning(APP_NAME, 'Stages
must be a number.'); return
        cell = r['cell'].split()[0]; info = repeater_info(self.lib, cell)
        args = ['-net', hn, '-prefix', r['prefix'], '-stages', stages,
'-auto_nets_create_with_prefix', '-inv_pair' if info and info[2] else
'-buff', cell]
        if only and r.get('loads', '').startswith('Only'): args +=
['-to_pins', sinks]
        res = self.gui_cmd('eco_add_repeater', args)
        if res: self.set_status('Inserted repeater chain: %s' % ', '.join(res))

    def build_bottom(self, nb):
        pf = ttk.Frame(nb); nb.add(pf, text='Problems')
        bar = ttk.Frame(pf, padding=(6, 3)); bar.pack(fill='x')
        for txt, var in (('Errors', self.f_err), ('Warnings',
self.f_warn), ('Notes', self.f_info)):
            ttk.Checkbutton(bar, text=txt, variable=var,
command=self.populate_problems).pack(side='left', padx=4)
        self.src_filter = tk.StringVar(value='All checks')
        sc = ttk.Combobox(bar, textvariable=self.src_filter,
state='readonly', width=34, values=['All checks', 'Same as verilator
--lint-only -Wall', 'Extra logic checks (beyond lint)', 'From
Verilator runs', 'Library checks'])
        sc.pack(side='left', padx=8); sc.bind('<<ComboboxSelected>>',
lambda e: self.populate_problems())
        self.prob_count = ttk.Label(bar, text='',
style='Hint.TLabel'); self.prob_count.pack(side='right')
        tw = ttk.Frame(pf); tw.pack(fill='both', expand=True)
        self.prob = ttk.Treeview(tw, columns=('code', 'where', 'msg',
'src'), show='tree headings', height=7); self.prob.heading('#0',
text=''); self.prob.column('#0', width=34, stretch=False)
        for c, t, w in (('code', 'Code', 150), ('where', 'Where',
190), ('msg', 'Message', 600), ('src', 'Source', 70)):
self.prob.heading(c, text=t); self.prob.column(c, width=w, minwidth=w
if c != 'msg' else 200, anchor='w', stretch=(c == 'msg'))
        ys = ttk.Scrollbar(tw, orient='vertical',
command=self.prob.yview); self.prob.configure(yscrollcommand=ys.set)
        self.prob.pack(side='left', fill='both', expand=True);
ys.pack(side='right', fill='y'); self.prob.bind('<Double-Button-1>',
self.on_problem_open); self.prob_rows = {}
        ef = ttk.Frame(nb); nb.add(ef, text='ECO log')
        bar2 = ttk.Frame(ef, padding=(6, 3)); bar2.pack(fill='x');
ttk.Button(bar2, text='Save ECO log…',
command=self.save_eco_log).pack(side='left')
        ttk.Label(bar2, text='Every ECO connection is recorded here,
including punched ports and new nets.',
style='Hint.TLabel').pack(side='left', padx=8)
      self.eco_text = tk.Text(ef, height=8, wrap='none',
font='TkFixedFont', relief='flat', borderwidth=0);
self.eco_text.pack(fill='both', expand=True)
        sf = ttk.Frame(nb); nb.add(sf, text='Verilog')
        bar3 = ttk.Frame(sf, padding=(6, 3)); bar3.pack(fill='x')
        ttk.Button(bar3, text='Apply edits to module',
command=self.apply_source).pack(side='left'); ttk.Button(bar3,
text='Reload', command=self.refresh_source).pack(side='left', padx=4)
        self.src_msg = ttk.Label(bar3, text='', style='Hint.TLabel');
self.src_msg.pack(side='left', padx=8)
        self.src_text = tk.Text(sf, height=8, wrap='none',
font='TkFixedFont', undo=True, relief='flat', borderwidth=0);
self.src_text.pack(fill='both', expand=True)
        vf = ttk.Frame(nb); nb.add(vf, text='Verilator')
        bar4 = ttk.Frame(vf, padding=(6, 3)); bar4.pack(fill='x')
        ttk.Button(bar4, text='Lint (-Wall)', command=lambda:
self.run_verilator(False)).pack(side='left')
        ttk.Button(bar4, text='Lint + smoke simulation',
command=lambda: self.run_verilator(True)).pack(side='left', padx=4)
        ttk.Button(bar4, text='Export bundle…',
command=self.export_bundle).pack(side='left')
        ttk.Label(bar4, text='Top:').pack(side='left', padx=(12, 2));
self.vl_top = ttk.Combobox(bar4, state='readonly', width=22);
self.vl_top.pack(side='left')
        self.vl_top.bind('<<ComboboxSelected>>', lambda e:
setattr(self, 'top', self.vl_top.get()))
        self.vl_status = ttk.Label(bar4, text='',
style='Hint.TLabel'); self.vl_status.pack(side='left', padx=8)
        self.vl_text = tk.Text(vf, height=8, wrap='none',
font='TkFixedFont', relief='flat', borderwidth=0);
self.vl_text.pack(fill='both', expand=True)
        nb.bind('<<NotebookTabChanged>>', lambda e:
self.refresh_source() if nb.index('current') == 2 else None)

    def bind_keys(self):
        r = self.root
        r.bind('<Control-o>', lambda e: self.open_netlist());
r.bind('<Control-l>', lambda e: self.open_lib());
r.bind('<Control-s>', lambda e: self.save_netlist())
        r.bind('<Control-z>', lambda e: self.do_undo());
r.bind('<Control-Z>', lambda e: self.do_redo()); r.bind('<Control-y>',
lambda e: self.do_redo())
        r.bind('<Control-f>', lambda e: self.open_find('inst'));
r.bind('<F3>', lambda e: self.next_match(1)); r.bind('<Shift-F3>',
lambda e: self.next_match(-1)); r.bind('<Control-q>', lambda e:
self.exit_now()); r.protocol('WM_DELETE_WINDOW', self.exit_now)
        def guarded(fn):
            def h(e):
                if isinstance(e.widget, (tk.Entry, ttk.Entry, tk.Text,
ttk.Combobox)): return
                fn()
            return h
        def arrow(dx, dy):
            def h(e):
                if isinstance(e.widget, (tk.Entry, ttk.Entry, tk.Text,
ttk.Combobox, ttk.Treeview, tk.Listbox, ttk.Spinbox)): return
                if not self.D: return
                st = 400 if (e.state & 0x1) else 80          # Shift = big step
                self.vx += dx * st / self.z; self.vy += dy * st /
self.z; self.request_redraw(); return 'break'
            return h
        for key, dx, dy in (('Left', -1, 0), ('Right', 1, 0), ('Up',
0, -1), ('Down', 0, 1)):
            r.bind('<%s>' % key, arrow(dx, dy)); r.bind('<Shift-%s>' %
key, arrow(dx, dy))
        r.bind('<Delete>', guarded(self.delete_selection));
r.bind('<Escape>', lambda e: self.cancel_pending())
        r.bind('<KeyPress-f>', guarded(self.fit_view));
r.bind('<KeyPress-z>', guarded(self.zoom_to_selection));
r.bind('<KeyPress-w>', guarded(self.toggle_wire))
        r.bind('<KeyPress-plus>', guarded(lambda: self.zoom_at(1.2)));
r.bind('<KeyPress-minus>', guarded(lambda: self.zoom_at(1 / 1.2)))

    # ---------------- helpers ----------------
    def set_status(self, t): self.status.configure(text=t)
    def tick_sysmon(self):
        try:
            t = time.time(); c = sum(os.times()[:2]); t0, c0 =
self._cpu_last; self._cpu_last = (t, c)
            pct = 100.0 * (c - c0) / max(1e-3, t - t0)
            self.sysmon.configure(text='RAM %d MB (peak %d)  ·  CPU
%d%%' % (cur_mb(), mem_mb(), pct))
       except Exception: pass
        self.root.after(1500, self.tick_sysmon)
    def cur_mod(self): return self.D.modules.get(self.mod) if self.D
and self.mod else None
    def cur_path(self): return tuple(i for i, _ in self.path)
    def toggle_wire(self): self.mode.set('select' if self.mode.get()
== 'wire' else 'wire'); self.on_mode()
    def on_mode(self):
        self.canvas.configure(cursor={'wire': 'crosshair', 'pan':
'fleur'}.get(self.mode.get(), 'arrow'))
        self.update_pending_status(); self.request_redraw()
    def toast(self, msg): self.set_status(msg)

    # ---------------- loading ----------------
    def open_lib(self):
        ps = filedialog.askopenfilenames(title='Open Liberty file(s) -
select several to merge them', filetypes=[('Liberty', '*.lib *.lib.gz
*.liberty *.gz'), ('All files', '*')])
        if ps: self.load_lib_files(list(self.root.tk.splitlist(ps)) if
isinstance(ps, str) else list(ps))
    def close_all(self, ask=True):
        if ask and self.D is not None and not
messagebox.askyesno(APP_NAME, 'Close the netlist and the libraries?
Unsaved ECO edits are lost.', icon='warning', default='no'): return
        self.D = None; self.lib = None; self.lib_name = '';
self.lib_files = []; self.lib_notes = []; self.lib_issues = [];
self.undo = []; self.redo = []; self.top = self.mod = self.root_mod =
None; self.path = []
        self.sel_node = self.sel_sig = None; self.issues = [];
self.flags_cache = None
        try: self.populate_library(); self.update_lib_info();
self.populate_problems(); self.refresh_all()
        except Exception: pass
        trim_memory(); self.set_status('Closed. Read the libraries
first (several can be added one after another), then the netlist.')
    def open_netlist(self):
        ps = filedialog.askopenfilenames(title='Open Verilog
netlist(s) - select several to merge them', filetypes=[('Verilog',
'*.v *.vg *.sv *.v.gz *.gz'), ('All files', '*')])
        if ps: self.load_netlist_files(list(self.root.tk.splitlist(ps))
if isinstance(ps, str) else list(ps))
    def install_design(self, D, name):
        self.D = D; self.v_name = name; self.undo = []; self.redo =
[]; self.vl_issues = []; self.last_export = None
        tops = find_tops(D)
        self.top = sorted(tops, key=lambda t: -hier_depth(D, t))[0] if
tops else D.order[0]
        self.root_mod = self.top; self.mod = self.top; self.path = [];
self.sel_node = self.sel_sig = None; self.pending = None
        self.invalidate_all(); self.need_fit = True;
self.refresh_all(); self.run_checks_now()
        try: self.update_lib_info()
        except Exception: pass
        if sum(len(m.insts) for m in D.modules.values()) > 50000: trim_memory()
    def load_sample(self):
        self.lib = parse_liberty(sample_lib_text()); self.lib_name =
'demo_tt_0p9v_25c.lib'; self.lib_issues = []; self.lib_files =
[self.lib_name]; self.lib_notes = []
        self.populate_library(); self.update_lib_info()
        self.install_design(parse_verilog(sample_netlist_text()),
'soc_top.v'); self.save_path = os.path.join(os.path.expanduser('~'),
'soc_top_eco.v')
        self.set_status('Sample loaded. Drag cells from the Library
tab onto the sheet, then use ECO wire to connect pins.')
    def invalidate_all(self):
        if self.D:
            for m in self.D.modules.values(): m._view = None; m._sym =
None; m._bc = None
        self.flags_cache = None

    # ---------------- navigation ----------------
    def open_module(self, name, steps=None, root=None):
        if not self.D or name not in self.D.modules: return
        if steps is None:
            steps, root = self.find_path(name)
        self.mod = name; self.path = list(steps); self.root_mod = root or name
        self.sel_node = self.sel_sig = None; self.need_fit = True;
self.flags_cache = None
        self.refresh_crumbs(); self.refresh_inspector();
self.sync_hier(); self.request_redraw(); self.update_pending_status()
    def find_path(self, name):
        D = self.D
        for root in [self.root_mod or self.top] + find_tops(D):
            if not root or root not in D.modules: continue
            if root == name: return [], root
            q = [(root, [])]; seen = {root}
            while q:
                cur, steps = q.pop(0)
                for i in child_insts(D, D.modules[cur]):
                    if True:
                        st = steps + [(i.name, i.type)]
                        if i.type == name: return st, root
                        if i.type not in seen: seen.add(i.type);
q.append((i.type, st))
        return [], name
    def refresh_crumbs(self):
        for w in self.crumbs.winfo_children(): w.destroy()
        if not self.D: ttk.Label(self.crumbs, text='No netlist
loaded', style='Hint.TLabel').pack(side='left'); return
        def link(text, cmd):
            b = ttk.Button(self.crumbs, text=text, command=cmd,
style='Toolbutton'); b.pack(side='left', padx=1)
        if self.path:
            link(self.root_mod, lambda:
self.open_module(self.root_mod, [], self.root_mod))
            for k, (inst, mod) in enumerate(self.path):
                ttk.Label(self.crumbs, text='/').pack(side='left')
                if k == len(self.path) - 1: ttk.Label(self.crumbs,
text='%s (%s)' % (inst, mod), font=(self.fam, 10,
'bold')).pack(side='left', padx=4)
                else:
                    steps = self.path[:k + 1]; link('%s (%s)' % (inst,
mod), lambda s=steps, m=mod: self.open_module(m, s, self.root_mod))
        else: ttk.Label(self.crumbs, text=self.mod, font=(self.fam,
10, 'bold')).pack(side='left', padx=4); ttk.Label(self.crumbs,
text='top-level view', style='Hint.TLabel').pack(side='left')
        m = self.cur_mod()
        if m: ttk.Label(self.crumbs, text='%d instances · %d ports' %
(len(m.insts), len(m.ports)), style='Hint.TLabel').pack(side='right')
        if m and is_paged(m):
            pg = getattr(m, '_page', 0); np_ = n_pages(m); a, b =
page_window(m, pg)
            bar = ttk.Frame(self.crumbs); bar.pack(side='right', padx=10)
            ttk.Button(bar, text='◀', width=3, command=lambda:
self.set_page(getattr(self.cur_mod(), '_page', 0) -
1)).pack(side='lef
            ttk.Label(bar, text=' page ').pack(side='left')
            e = ttk.Entry(bar, width=7, justify='center'); e.insert(0,
str(pg + 1)); e.pack(side='left')
            e.bind('<Return>', lambda ev: self.set_page(int(e.get()) -
1 if e.get().strip().isdigit() else pg))
            ttk.Label(bar, text=' of %d  (cells %d–%d)  ' % (np_, a +
1, b)).pack(side='left')
            ttk.Button(bar, text='▶', width=3, command=lambda:
self.set_page(getattr(self.cur_mod(), '_page', 0) +
1)).pack(side='left'   # ---------------- view and drawing ----------------
    def on_view_change(self):
        self.invalidate_all(); self.refresh_inspector(); self.request_redraw()
    def relayout(self):
        m = self.cur_mod()
        if m: m.layout = {}; m._view = None; self.need_fit = True;
self.request_redraw()
    def request_redraw(self):
        if self.redraw_pending: return
        self.redraw_pending = True; self.root.after(8, self.redraw)
    def issue_flags(self):
        if self.flags_cache and self.flags_cache[0] == self.mod:
return self.flags_cache[1]
        rank = {'error': 3, 'warn': 2, 'info': 1}; fl = {}
        for i in self.issues + self.vl_issues:
            if i.mod != self.mod or not i.inst: continue
            k = 'i:' + i.inst
            if k not in fl or rank[i.sev] > rank[fl[k]]: fl[k] = i.sev
        self.flags_cache = (self.mod, fl); return fl
    def draw_grid(self, W, H):
        z = self.z; c = self.canvas; T = self.T
        for step, col in ((20, T['grid']), (100, T['grid_major'])):
            px = step * z
            if px < 7: continue
            x0 = math.floor(self.vx / step) * step
            x = x0
            while (x - self.vx) * z <= W:
                sx = (x - self.vx) * z; c.create_line(sx, 0, sx, H,
fill=col, tags='grid'); x += step
            y = math.floor(self.vy / step) * step
            while (y - self.vy) * z <= H:
                sy = (y - self.vy) * z; c.create_line(0, sy, W, sy,
fill=col, tags='grid'); y += step
        c.tag_lower('grid')

    def fit_view(self): self.need_fit = True; self.request_redraw()
    def to_world(self, x, y): return self.vx + x / self.z, self.vy + y / self.z
    def draw_rubber(self):
        wd = self.wire_drag; nd = wd['node']; q = pin_pos(nd, wd['pin'])
        if not q: return
        x0 = (q[0] - self.vx) * self.z; y0 = (q[1] - self.vy) * self.z
        self.canvas.create_line(x0, y0, wd['x'], wd['y'],
fill=self.T['acc'], width=2, dash=(5, 3), tags='rubber')

    # ---------------- mouse ----------------
    def pan_start(self, e): self.pan = (e.x, e.y, self.vx, self.vy)
    def pan_move(self, e):
        if self.pan: self.vx = self.pan[2] - (e.x - self.pan[0]) /
self.z; self.vy = self.pan[3] - (e.y - self.pan[1]) / self.z;
self.request_redraw()
    def hpin_of(self, nd, pin):
        if nd.kind == 'port': return HPin(self.cur_path(), None, nd.ref.name)
        if nd.kind == 'inst': return HPin(self.cur_path(), nd.ref.name, pin)
        return None
    def on_press(self, e):
        self.canvas.focus_set()
        if not self.D: return
        v = self.get_view(); wx, wy = self.to_world(e.x, e.y)
        if getattr(self, 'placing', None):
self.place_instance(self.placing, wx, wy); self.placing = None;
self.canvas.configure(cursor='arrow'); return
        h = hit_test(v, wx, wy, self.z); ctrl = bool(e.state & 0x4)
        self.press = {'x': e.x, 'y': e.y, 'hit': h, 'moved': False,
'shift': bool(e.state & 0x1), 'ctrl': ctrl, 'alt': bool(e.state &
0x8)}
        if self.mode.get() == 'pan' or self.press['alt']: self.pan =
(e.x, e.y, self.vx, self.vy); self.press['hit'] = h = None;
self.press['panonly'] = True; return
        if h and h[0] == 'node' and not ctrl:
            nd = h[1]
            if not (self.msel and self.node_in_msel(nd)):
self.select_node(nd.key)
            self.node_drag = {'nd': nd, 'wx': wx, 'wy': wy, 'ox':
nd.x, 'oy': nd.y}
        elif h is None: self.band = {'x': e.x, 'y': e.y}
    def on_double(self, e):
        if not self.D: return
        v = self.get_view(); wx, wy = self.to_world(e.x, e.y); h =
hit_test(v, wx, wy, self.z)
        if h and h[0] == 'node' and h[1].kind == 'inst' and h[1].rt[0]
== 'module':
            inst = h[1].ref; self.open_module(inst.type, self.path +
[(inst.name, inst.type)], self.root_mod)
        elif v.big and v.overview: self.zoom_at(5.0, e.x, e.y)
    def on_hover(self, e):
        wx, wy = self.to_world(e.x, e.y);
self.coords.configure(text='x %.0f  y %.0f   zoom %d%%' % (wx, wy,
self.z * 100))
        if not self.D or self.node_drag or self.wire_drag: return
        v = self.get_view(); h = hit_test(v, wx, wy, self.z) if v else None
        if h and h[0] == 'pin':
            hp = self.hpin_of(h[1], h[2]); sids = h[1].pin_sigs.get(h[2])
            self.set_status('Pin %s   net: %s' % (hpin_str(hp) if hp
else h[2], ', '.join(sids) if sids else '(not connected)'))
            self.canvas.configure(cursor='crosshair')
        elif not getattr(self, 'placing', None):
self.canvas.configure(cursor='crosshair' if self.mode.get() == 'wire'
else 'arrow')
    # ---------------- selection ----------------
    def selection_found(self):
        """everything currently selected in the schematic as Found
objects: the multi-selection, or else the single clicked object"""
        if self.msel: return list(self.msel)
        if not self.D: return []
        out = []; cur = self.cur_path(); v = self.get_view()
        if self.sel_pin:
            out.append(Found('pin', cur, self.sel_pin[0], self.sel_pin[1]))
            if v is not None and self.sel_sig and self.sel_sig in
v.sigs and not v.sigs[self.sel_sig].stub: out.append(Found('net', cur,
self.sel_sig, ''))
        elif v is not None and self.sel_node and self.sel_node in v.by_key:
            f = self.found_of_node(v.by_key[self.sel_node])
            if f: out.append(f)
        elif v is not None and self.sel_sig and self.sel_sig in v.sigs
and not v.sigs[self.sel_sig].stub:
            out.append(Found('net', cur, self.sel_sig, ''))
        return out
    def drop_msel(self):
        self.sel_pin = None
        if self.msel: self.msel = None; self.hl_ver += 1
    def select_node(self, key): self.drop_msel(); self.sel_node = key;
self.sel_sig = None; self.sel_cell = None; self.refresh_inspector();
self.request_redraw(); self.sync_layout_sel()
    def select_sig(self, sid): self.drop_msel(); self.sel_sig = sid;
self.sel_node = None; self.sel_cell = None; self.refresh_inspector();
self.request_redraw()
    def select_none(self):
        if self.sel_node or self.sel_sig or self.sel_cell or self.msel
or self.sel_pin: self.drop_msel(); self.sel_node = self.sel_sig =
None; self.sel_cell = None; self.refresh_inspector();
self.request_redraw(); self.sync_layout_sel()
    def found_of_node(self, nd):
        cur = self.cur_path()
        return Found('inst', cur, nd.ref.name, '') if nd.kind ==
'inst' else Found('topport', cur, nd.ref.name, '') if nd.kind ==
'port' else None
    def node_in_msel(self, nd):
        f = self.found_of_node(nd); return f is not None and f in
(self.msel or [])
    def toggle_hit(self, h):
        cur = self.cur_path()
        if h[0] == 'sig':
            f = Found('net', cur, h[1].id, '') if not h[1].stub else None
        else: f = self.found_of_node(h[1])
        if f is None: return
        sel = list(self.msel or [])
        if not sel:                      # fold the single
click-selection into the set
            if self.sel_node and self.sel_node in self.get_view().by_key:
                g = self.found_of_node(self.get_view().by_key[self.sel_node])
                if g: sel.append(g)
            elif self.sel_sig and self.sel_sig in
self.get_view().sigs: sel.append(Found('net', cur, self.sel_sig, ''))
        if f in sel: sel.remove(f)
        else: sel.append(f)
        self.set_msel(sel)
    def band_select(self, b, e, additive):
        v = self.get_view()
        if v is None or (v.big and v.overview): self.set_status('Zoom
in to rubber-band select (overview shows no individual cells).');
return
        x0, y0 = self.to_world(min(b['x'], e.x), min(b['y'], e.y));
x1, y1 = self.to_world(max(b['x'], e.x), max(b['y'], e.y)); cur =
self.cur_path()
        cand = nodes_in(v, (x0, y0, x1, y1)) if v.big else v.nodes; out = []
        for nd in cand:
            if nd.x <= x1 and nd.x + nd.sym.w >= x0 and nd.y <= y1 and
nd.y + nd.sym.h >= y0:
                f = self.found_of_node(nd)
                if f: out.append(f)
        if len(out) < 3000:
            for s in v.sig_list:
                if s.stub: continue
                ensure_geom(v, s)
                if s.polys and s.bb[0] >= x0 and s.bb[2] <= x1 and
s.bb[1] >= y0 and s.bb[3] <= y1: out.append(Found('net', cur, s.id,
''))
        if additive:
            keep = list(self.msel or []); out = keep + [f for f in out
if f not in keep]
        if out: self.set_msel(out)
        else:
            if not additive: self.select_none()
            self.set_status('Nothing inside the selection box.')

    # ---------------- ECO click workflow ----------------
    def set_pending(self, hp):
        self.pending = hp; self.update_pending_status(); self.request_redraw()
    def cancel_pending(self):
        had = self.pending is not None; self.pending = None;
self.placing = None; self.wire_drag = None
        self.canvas.configure(cursor='arrow');
self.update_pending_status(); self.request_redraw()
        if had: self.set_status('ECO connection cancelled.')
    def complete_pending(self, hp):
        a = self.pending
        if a == hp: self.cancel_pending(); return
        self.pending = None; self.do_eco_connect(a, hp)
    def ask_uniquify(self, modname, n):
        r = messagebox.askyesnocancel('Shared module', 'Module "%s" is
instantiated %d times.\n\nYes  = make a private copy for this instance
(uniquify)\nNo   = add the new port to every instance (the others get
an unconnected port)\nCancel = abort this ECO.' % (modname, n),
parent=self.root)
        return 'cancel' if r is None else ('uniquify' if r else 'all')
    def ask_bit(self, name, lo, hi):
        r = FormDialog(self.root, 'Which bit?', [('b', 'The pin is 1
bit wide but "%s" has %d bits.\nBit to use:' % (name, hi - lo + 1),
str(lo), [str(i) for i in range(lo, hi + 1)])], 'Use this bit').result
        if not r: return None
        try: return int(r['b'])
        except ValueError: return None
    def eco_simple(self, what, hp):
        D = self.D; lib = self.lib; root = self.root_mod
        names = affected_modules(D, root, hp)
        fn = {'disconnect': lambda: (eco_disconnect(D, lib, root, hp),
[]), 'tie0': lambda: (eco_tie(D, lib, root, hp, 0), []), 'tie1':
lambda: (eco_tie(D, lib, root, hp, 1), [])}[what]
        if self.edit('ECO ' + what, names, fn): self.refresh_eco_log()

    # ---------------- edit core + undo ----------------
    def edit(self, label, names, fn, raise_errors=False):
        global JRN
        before = self.snapshot(names)
        jr = Journal() if any(b is BIGMARK for b in
before['mods'].values()) else None
        JRN = jr
        try: res = fn()
        except Exception as ex:
            JRN = None; self.restore(before)
            if jr: jr.swap()
            self.invalidate_all()
            if raise_errors: raise
            if isinstance(ex, EcoError):
messagebox.showwarning(APP_NAME, str(ex))
            else: messagebox.showerror(APP_NAME, 'Edit failed and was
rolled back:\n%s' % ex)
            return None
        JRN = None
        created = res[1] if isinstance(res, tuple) and len(res) == 2
and isinstance(res[1], list) else []
        for c in created: before['mods'].setdefault(c, None)
        self.undo.append({'label': label, 'before': before, 'jrn':
jr}); self.undo = self.undo[-(60 if jr is None else 12):]; self.redo =
[]
        if self.batch_depth: self._need_after = True
        else: self.after_edit()
        return res if res is not None else True
    def batch_begin(self): self.batch_depth += 1
    def batch_end(self):
        self.batch_depth = max(0, self.batch_depth - 1)
        if self.batch_depth == 0 and self._need_after:
self._need_after = False; self.after_edit(); self.refresh_eco_log()
    def confirm_noneq(self, msg):
        return messagebox.askyesno('Non-equivalent footprint', msg +
'\n\nSwap anyway?', icon='warning', default='no', parent=self.root)
    def after_edit(self):
        self.invalidate_all(); v = self.get_view()
        if self.sel_node and (v is None or self.sel_node not in
v.by_key): self.sel_node = None
        if self.sel_sig and (v is None or self.sel_sig not in v.sigs):
self.sel_sig = None
        self.refresh_all(); self.schedule_check()
    def _flip(self, src, dst, verb):
        e = src.pop(); cur =
self.snapshot(list(e['before']['mods'].keys())); jr = e.get('jrn')
        dst.append({'label': e['label'], 'before': cur, 'jrn': jr})
        self.restore(e['before'])
        if jr: jr.swap()
        self.after_edit(); self.refresh_eco_log();
self.set_status('%s: %s' % (verb, e['label']))
    def do_undo(self):
        if self.undo: self._flip(self.undo, self.redo, 'Undid')
    def do_redo(self):
        if self.redo: self._flip(self.redo, self.undo, 'Redid')

    # ---------------- refresh helpers ----------------
    def refresh_all(self):
        self.refresh_crumbs(); self.populate_hier();
self.refresh_inspector(); self.populate_problems();
self.refresh_eco_log()
        self.b_undo.state(['!disabled'] if self.undo else
['disabled']); self.b_redo.state(['!disabled'] if self.redo else
['disabled'])
        if self.D: self.vl_top['values'] = self.D.order;
self.vl_top.set(self.top or '')
        self.request_redraw()
    def update_lib_info(self):
        if not self.lib: self.lib_info.configure(text='No Liberty file
loaded.'); return
        s = self.lib.stats; self.lib_info.configure(text='%s\n%d cells
· NLDM %d · CCS %d · ECSM %d%s' % (self.lib_name, s['cells'],
s['nldm'], s['ccs'], s['ecsm'], ('\n' + ', '.join(self.lib_files) + ('
  [locked]' if self.D is not None else '')) if len(self.lib_files) > 1
else ('   [locked]' if self.D is not None else '')))

    # ---------------- hierarchy tree ----------------
    def on_hier_select(self, e):
        if self.hier_lock: return
        sel = self.hier.selection()
        if sel and sel[0] in self.hier_map           mn, steps, root = self.hier_map[sel[0]];
self.open_module(mn, steps, root)

    # ---------------- library panel + drag and drop ----------------
    def populate_library(self):
        tv = self.libtree; tv.delete(*tv.get_children())
        q = self.lib_q.get().strip().lower(); cat =
self.lib_cat.get(); want = {'Combinational': 'comb', 'Sequential':
'seq', 'Macros': 'macro'}.get(cat)
        if self.D and cat in ('All kinds', 'Modules'):
            g = tv.insert('', 'end', iid='g_mod', text='Design
modules', open=True)
            for mn in self.D.order:
                if q in mn.lower(): tv.insert(g, 'end', iid='m:' + mn,
text=mn, values=('module · %d instances' %
len(self.D.modules[mn].insts), ''))
        if self.lib and cat != 'Modules':
            g = tv.insert('', 'end', iid='g_cell', text='Library
cells', open=True); n = 0
            for name in self.lib.order:
                if q and q not in name.lower(): continue
                c = self.lib.cells[name]
                if want andclassify_cell(c)['cat'] != want: continue
                tv.insert(g, 'end', iid='c:' + name, text=name,
values=(describe_cell(c), c.model)); n += 1
                if n >= 1500: tv.insert(g, 'end', text='… refine the
search to see more', values=('', '')); break
    def on_lib_select(self, e):
        sel = self.libtree.selection()
        if not sel or sel[0][:2] not in ('c:', 'm:'): return
        self.sel_cell = sel[0]; self.sel_node = self.sel_sig = None;
self.draw_preview(); self.refresh_inspector()self.request_redraw()
    def lib_symbol(self, iid):
        kind, name = iid[0], iid[2:]
        if kind == 'c' and self.lib and name in self.lib.cells: return
cell_symbol(self.lib.cells[name], self.v_block.get())
        if kind == 'm' and self.D and name in self.D.modules: return
module_symbol(self.D.modules[name])
        return None
    def draw_preview(self):
        c = self.prev; c.delete('all')
        if not self.sel_cell: return
        sym = self.lib_symbol(self.sel_cell)
        if sym is None: return
        W = max(c.winfo_width(), 200); H = max(c.winfo_height(), 100);
z = min(1.8, (W - 40) / sym.w, (H - 30) / sym.h)
        P = TkPainter(c, -(W / z - sym.w) / 2, -(H / z - sym.h) / 2,
z, self.T, self.fam); paint_sym(P, sym, 0, 0, self.T)
    def dnd_press(self, e):
        iid = self.libtree.identify_row(e.y)
        self.dnd = {'name': iid[2:], 'x': e.x_root, 'y': e.y_root,
'ghost': None} if iid and iid[:2] in ('c:', 'm:') else None
        if self.dnd: self.dnd['iid'] = iid
    def dnd_motion(self, e):
        d = self.dnd
        if not d: return
        if d['ghost'] is None:
            if abs(e.x_root - d['x']) + abs(e.y_root - d['y']) < 8: return
            g = tk.Toplevel(self.root); g.overrideredirect(True)
            try: g.attributes('-topmost', True)
            except tk.TclError: pass
            sym = self.lib_symbol(d['iid'])
            if sym is None: g.destroy(); self.dnd = None; return
            z = 0.9; cv = tk.Canvas(g, width=int(sym.w * z + 24),
height=int(sym.h * z + 34), bg=self.T['bg'], highlightthickness=1,
highlightbackground=self.T['acc']); cv.pack()
            P = TkPainter(cv, -12 / z, -6 / z, z, self.T, self.fam);
paint_sym(P, sym, 0, 0, self.T)
            cv.create_text(6, int(sym.h * z + 22), text=d['name'],
anchor='w', fill=self.T['ink'], font=(self.fam, -11, 'bold'))
            d['ghost'] = g; self.canvas.configure(cursor='plus')
        d['ghost'].geometry('+%d+%d' % (e.x_root + 14, e.y_root + 10))
        over = self.root.winfo_containing(e.x_root, e.y_root)
        self.set_status('Drop on the sheet to add %s' % d['name'] if
over is self.canvas else 'Drag %s onto the schematic sheet' %
d['name'])
    def dnd_release(self, e):
        d = self.dnd; self.dnd = None
        if not d or d['ghost'] is None: return
        d['ghost'].destroy(); self.canvas.configure(cursor='arrow')
        over = self.root.winfo_containing(e.x_root, e.y_root)
        if over is self.canvas:
            wx, wy = self.to_world(e.x_root -
self.canvas.winfo_rootx(), e.y_root - self.canvas.winfo_rooty());
self.place_instance(d['name'], wx, wy)
        else: self.set_status('Cancelled: drop the cell on the
schematic sheet.')
    def lib_double(self, e):
        iid = self.libtree.identify_row(e.y)
        if iid and iid[:2] in ('c:', 'm:'): self.place_at_center(iid[2:])
    def place_at_center(self, type_):
        if not self.D: messagebox.showinfo(APP_NAME, 'Load a netlist
first.'); return
        W = self.canvas.winfo_width(); H = self.canvas.winfo_height();
k = getattr(self, 'stagger', 0); self.stagger = (k + 1) % 8
        wx, wy = self.to_world(W / 2 + k * 24 * self.z, H / 2 + k * 24
* self.z); self.place_instance(type_, wx, wy)

    # ---------------- adding / deleting instances ----------------
    def place_instance(self, type_, wx, wy):
        m = self.cur_mod()
        if m is None: messagebox.showinfo(APP_NAME, 'Load a netlist
first.'); return
        if type_ == m.name: messagebox.showwarning(APP_NAME, 'A module
cannot instantiate itself.'); return
        rt = resolve_type(self.D, self.lib, type_); fake = Inst('', type_)
        sym = sym_for_inst(self.D, self.lib, fake, rt, self.v_block.get())
        paged = is_paged(m); idx = page_window(m, getattr(m, '_page',
0))[1] - 1 if paged else None
        def fn():
            inst = add_instance(m, type_); inst.prim = rt[0] == 'prim'
            if m.layout is None: m.layout = {}
            if paged: move_inst_to(m, inst, idx)       # keep it on
the page being viewed
            m.layout['i:' + inst.name] = (snap_to(wx - sym.w / 2),
snap_to(wy - sym.h / 2)); return inst
        inst = self.edit('add ' + type_, [m.name], fn)
        if inst and inst is not True: self.select_node('i:' +
inst.name); self.set_status('Added %s (%s). Drag from its pins, or use
ECO wire, to connect it.' % (inst.name, type_))
    def add_instance_dialog(self, wx, wy):
        names = (list(self.D.order) if self.D else []) +
(self.lib.order[:3000] if self.lib else [])
        r = FormDialog(self.root, 'Add instance', [('t', 'Cell or
module', '', names)], 'Place').result
        if r and r['t'].strip(): self.place_instance(r['t'].strip(), wx, wy)
    def delete_selection(self):
        if self.msel:
            insts = [found_str(f).lstrip('/') for f in self.msel if
f.kind == 'inst']; nets = [found_str(f).lstrip('/') for f in self.msel
if f.kind == 'net']
            if len(insts) + len(nets) > 1 or (insts or nets):
                if not messagebox.askyesno(APP_NAME, 'Delete %d
instance(s) and %d wire(s) from the selection?' % (len(insts),
len(nets)), parent=self.root): return
                self.batch_begin()
                try:
                    if insts: self.gui_cmd('delete_inst', [insts])
                    if nets: self.gui_cmd('delete_net', [nets])
                finally: self.batch_end()
                self.msel = None; self.hl_ver += 1;
self.request_redraw(); return
        m = self.cur_mod(); v = self.get_view()
        if not m or not self.sel_node or self.sel_node not in v.by_key: return
        nd = v.by_key[self.sel_node]
        if nd.kind == 'inst':
            def fn(): remove_instance(m, nd.ref.name);
m.layout.pop(nd.key, None)
            self.edit('delete ' + nd.ref.name, [m.name], fn)
        elif nd.kind == 'assign':
            def fn2(): jlist(m, 'assigns'); m.assigns.remove(nd.ref);
touch(m); [m.layout.pop(k) for k in [k for k in m.layout if
k.startswith('a:')]]
            self.edit('delete assign', [m.name], fn2)
        elif nd.kind == 'port':
            names = [m.name] + [pm.name for pm, pi in
parents_of(self.D, m.name)]
            self.edit('delete port ' + nd.ref.name, names, lambda:
remove_port(self.D, m, nd.ref.name))
        self.sel_node = None
    def duplicate_selected(self):
        m = self.cur_mod(); v = self.get_view(); nd =
v.by_key.get(self.sel_node) if v else None
        if not nd or nd.kind != 'inst': return
        def fn():
            ni = add_instance(m, nd.ref.type, re.sub(r'_\d+$', '',
nd.ref.name)); ni.params = nd.ref.params; ni.prim = nd.ref.prim
            m.layout['i:' + ni.name] = (nd.x + 30, nd.y + nd.sym.h +
34); return ni
        ni = self.edit('duplicate ' + nd.ref.name, [m.name], fn)
        if ni and ni is not True: self.select_node('i:' + ni.name)

    # ---------------- inspector ----------------
    def hdr(self, text):
        ttk.Label(self.insp, text=text, font=(self.fam, 11,
'bold')).pack(fill='x', pady=(8, 3))
    def entry_row(self, label, value, commit, values=None, width=26):
        f = ttk.Frame(self.insp); f.pack(fill='x', pady=2);
ttk.Label(f, text=label, width=9).pack(side='left')
        var = tk.StringVar(value=value)
        w = ttk.Combobox(f, textvariable=var, values=values,
width=width) if values else ttk.Entry(f, textvariable=var,
width=width)
        w.pack(side='left', fill='x', expand=True)
        def go(ev=None):
            if var.get() != value: commit(var.get())
        w.bind('<Return>', go); w.bind('<FocusOut>', go); return w
    def make_table(self, cols, heights=6):
        f = ttk.Frame(self.insp); f.pack(fill='both', pady=3)
        tv = ttk.Treeview(f, columns=[c[0] for c in cols[1:]],
show='tree headings', height=heights)
        tv.heading('#0', text=cols[0][1]); tv.column('#0', width=cols[0][2])
        for k, t, w in cols[1:]: tv.heading(k, text=t); tv.column(k, width=w)
        ys = ttk.Scrollbar(f, orient='vertical', command=tv.yview);
tv.configure(yscrollcommand=ys.set); tv.pack(side='left', fill='both',
expand=True); ys.pack(side='right', fill='y'); return tv
    def tree_edit(self, tv, iid, column, commit):
        bb = tv.bbox(iid, column)
        if not bb: return
        x, y, w, h = bb; val = tv.item(iid, 'text') if column == '#0'
else tv.set(iid, column)
        e = ttk.Entry(tv); e.place(x=x, y=y, width=max(w, 60),
height=h); e.insert(0, val); e.select_range(0, 'end'); e.focus_set();
done = [False]
        def finish(save):
            if done[0]: return
            done[0] = True; v = e.get(); e.destroy()
            if save and v != val: commit(v)
        e.bind('<Return>', lambda ev: finish(True));
e.bind('<Escape>', lambda ev: finish(False)); e.bind('<FocusOut>',
lambda ev: finish(True))
    def sym_canvas(self, sym, h=110):
        c = tk.Canvas(self.insp, height=h, bg=self.T['bg'],
highlightthickness=1, highlightbackground=self.T['grid_major']);
c.pack(fill='x', pady=4)
        def draw(ev=None):
            c.delete('all'); W = max(c.winfo_width(), 200); z =
min(1.8, (W - 30) / sym.w, (h - 22) / sym.h)
            P = TkPainter(c, -(W / z - sym.w) / 2, -(h / z - sym.h) /
2, z, self.T, self.fam); paint_sym(P, sym, 0, 0, self.T)
        c.bind('<Configure>', draw)
    def insp_inst(self, nd):
        inst = nd.ref; rt = nd.rt; m = self.cur_mod(); pins =
type_pins(rt, inst)
        self.hdr('Instance')
        self.entry_row('Name', inst.name, lambda nv:
self.rename_inst(inst.name, nv.strip()))
        types = (list(self.D.order) if self.D else []) +
(self.lib.order[:3000] if self.lib else [])
        self.entry_row('Type', inst.type, lambda nv:
self.retype_inst(inst.name, nv.strip()), values=types)
        kind = {'cell': 'Library cell · ' + (dscribe_cell(rt[1]) if
rt[0] == 'cell' else ''), 'module': 'Module · %d instances inside' %
len(rt[1].insts) if rt[0] == 'module' else '',
                'prim': 'Verilog primitive', 'unknown': 'Not found in
the netlist or Liberty file'}[rt[0]]
        ttk.Label(self.insp, text=kind, style='Err.TLabel' if rt[0] ==
'unknown' else 'Hint.TLabel', wraplength=300).pack(anchor='w')
        if rt[0] == 'cell': ttk.Label(self.insp, text='Timing: %s   ·
 area %s' % (rt[1].model, rt[1].area),
style='Hint.TLabel').pk(anchor='w')
        self.sym_canvas(nd.sym)
        ttk.Label(self.insp, text='Pin connections (double-click a net
to edit)', style='Hint.TLabel').pack(anchor='w')
        tv = self.make_table([('pin', 'Pin', 110), ('dir', '', 38),
('net', 'Connected to', 170)], 9)
        conns = dict(named_conns(inst, pins))
        for p in pins:
            ex = conns.get(p.name) or ''; bad = bool(ex) and
bool(bits_of(m, ex).undeclared)
            tv.insert('', 'end', iid=p.name, text=p.name + ('[%d:%d]'
% (p.msb, p.lsb) if p.width > 1 else ''), values=('◀' if p.dir ==
'input' else '▶' if p.dir == 'output' else '◆', ex), tags=('bad',) if
bad else ())
        tv.tag_configure('bad', foreground=self.T['bad']); widths =
{p.name: p.width for p in pins}
        tv.bind('<Double-Button-1>', lambda e: self.tree_edit(tv,
tv.identify_row(e.y), 'net', lambda nv, i=tv.identify_row(e.y):
self.edit_pin(inst.name, i, nv, widths.get(i, 1))) if
tv.identify_row(e.y) and tv.identify_column(e.x) == '#2' else None)
        f = ttk.Frame(self.insp); f.pack(fill='x', pady=6)
        if rt[0] == 'module': ttk.Button(f, text='Go inside',
command=lambda: self.open_module(inst.type, self.path + [(inst.name,
inst.type)], self.root_mod)).pack(side='left', padx=2)
        ttk.Button(f, text='Duplicate',
command=self.duplicate_selected).pack(side='left', padx=2);
ttk.Button(f, text='Delete',
command=self.delete_selection).pack(side='left', padx=2)
        ttk.Label(self.insp, text='Tip: drag from one pin to another
(or use ECO wire) to connect; the net is named automatically.',
style='Hint.TLabel', wraplength=300).pack(anchor='w', pady=4)
    def insp_port(self, nd):
        p = nd.ref; m = self.cur_mod(); n = m.nets.get(p.name)
        self.hdr('Port'); self.entry_row('Name', p.name, lambda nv:
self.rename_net(p.name, nv.strip()))
        self.entry_row('Direction', p.dir, lambda nv:
self.set_shape(p.name, '', nv.strip()), values=['input', 'output',
'inout'])
        self.entry_row('Width', ('%d:%d' % (n.msb, n.lsb)) if n and
n.vec else '', lambda nv: self.set_shape(p.name, nv.strip(), None))
        par = len(parents_of(self.D, m.name))
        ttk.Label(self.insp, text='Changing a port updates the %d
instance%s of %s in parent modules.' % (par, '' if par == 1 else 's',
m.name), style='Hint.TLabel', wraplength=300).pack(anchor='w', pady=4)
        ttk.Button(self.insp, text='Delete port',
command=self.delete_selection).pack(anchor='w')
    def insp_assign(self, nd):
        m = self.cur_mod(); self.hdr('Assign')
        self.entry_row('Target', nd.ref.lhs, lambda nv:
self.edit_assign(nd.ref, 'lhs', nv)); self.entry_row('Source',
nd.ref.rhs, lambda nv: self.edit_assign(nd.ref, 'rhs', nv))
        ttk.Button(self.insp, text='Delete assign',
command=self.delete_selection).pack(anchor='w', pady=6)
    def insp_net(self, s):
        m = self.cur_mod(); n = m.nets.get(s.base) if s.base else None
        self.hdr('Net'); ttk.Label(self.insp, text=s.label + ('   [%d
bits]' % s.width if s.width > 1 else ''), font=(self.mono,
11)).pack(anchor='w')
        if n:
            self.entry_row('Rename', s.base, lambda nv:
self.rename_net(s.base, nv.strip()))
            ttk.Label(self.insp, text=('Port (%s)' % n.dir if n.dir
else 'Wire') + (' · range [%d:%d]' % (n.msb, n.lsb) if n.vec else ' ·
single bit'), style='Hint.TLabel').pack(anchor='w')
        elif s.stub: ttk.Label(self.insp, text='Expression or
constant. Edit it from the pin it is connected to.',
style='Hint.TLabel', wraplength=300).pack(anchor='w')
        else: ttk.Label(self.insp, text='Usebut never declared
(implicit net).', style='Err.TLabel').pack(anchor='w')
        v = self.get_view()
        for title, sel in (('Drivers', ('out', 'io')), ('Loads', ('in', 'io'))):
            rows = [(nd, pin) for nd, pin, d in s.pins if d in sel]
            self.hdr('%s (%d)' % (title, len(rows))); lb =
tk.Listbox(self.insp, height=min(5, max(1, len(rows))),
activestyle='none', relief='flat', bg=self.T['fill'],
fg=self.T['ink'], selectbackground=self.T['acc_soft'],
selectforeground=self.T['ink'], highlightthickness=1,
highlightbackground=self.T['grid_major'])
            for nd, pin in rows: lb.insert('end', ('port ' +
nd.ref.name) if nd.kind == 'port' else '%s.%s' % (nd.ref.name if
nd.kind == 'inst' else 'assign', pin))
            lb.pack(fill='x'); lb.bind('<Double-Button-1>', lambda e,
rows=rows, lb=lb: self.select_node(rows[lb.curselection()[0]][0].key)
if lb.curselection() else None)
        if n and not n.dir: ttk.Button(self.insp, text='Delete wire
and disconnect its pins', command=lambda: self.edit('delete wire ' +
s.base, [m.name], lambda: remove_net(m, s.base))).pack(anchor='w',
pady=8)
    def insp_module(self, m):
        self.hdr('Module'); self.entry_row('Name', m.name,
self.rename_module_cb)
        par = len(parents_of(self.D, m.name))
        ttk.Label(self.insp, text='%d instances · %d assigns · used %d
time%s · %s' % (len(m.insts), len(m.assigns), par, '' if par == 1 else
's', 'edited' if m.dirty else 'written back exactly as loaded'),
style='Hint.TLabel', wraplength=300).pack(anchor')
        if m.raw: ttk.Label(self.insp, text='%d behavioral block(s)
kept verbatim, not drawn.' % len(m.raw), style='Err.TLabel',
wraplength=300).pack(anchor='w')
        self.hdr('Ports'); tv = self.make_table([('n', 'Port', 120),
('d', 'Dir', 62), ('w', 'Width', 70)], 6)
        for p in m.ports:
            n = m.nets.get(p.name); tv.insert('', 'end', iid='p:' +
p.name, text=p.name, values=(p.dir, ('%d:%d' % (n.msb, n.lsb)) if n
and n.vec else '1'))
        def pedit(e):
            iid = tv.identify_row(e.y); col = tv.identify_column(e.x)
            if not iid: return
            name = iid[2:]
            if col == '#0': self.tree_edit(tv, iid, '#0', lambda nv:
self.rename_net(name, nv.strip()))
            elif col == '#1': self.tree_edit(tv, iid, 'd', lambda nv:
self.set_shape(name, '', nv.strip()))
            else: self.tree_edit(tv, iid, 'w', lambda nv:
self.set_shape(name, nv.strip(), None))
        tv.bind('<Double-Button-1>', pedit)
        f = ttk.Frame(self.insp); f.pack(fill='x'); ttk.Button(f,
text='Add port…', command=self.add_port_dialog).pack(side='left',
padx=2)
        ttk.Button(f, text='Delete port', command=lambda:
self.del_from_table(tv, 'port')).pack(side='left', padx=2)
        self.hdr('Wires'); tw = self.make_table([('n', 'Wire', 150),
('w', 'Width', 80)], 5)
        shown = 0
        for nm in m.net_order:
            n = m.nets.get(nm)
            if n and not n.dir:
                if shown >= WIRE_ROWS_MAX:
                    tw.insert('', 'end', iid='x:more', text='… %re
wires not listed (use Find, or get_nets in the console)' % max(1,
len(m.nets) - len(m.ports) - shown), values=('',)); break
                tw.insert('', 'end', iid='w:' + nm, text=nm,
values=(('%d:%d' % (n.msb, n.lsb)) if n.vec else '1',)); shown += 1
        def wedit(e):
            iid = tw.identify_row(e.y)
            if not iid or iid.startswith('x:'): return
            name = iid[2:]
            if tw.identify_column(e.x) == '#0': self.tree_edit(tw,
iid, '#0', lambda nv: self.rename_net(name, nv.strip()))
            else: self.tree_edit(tw, iid, 'w', lambda nv:
self.set_shape(name, nv.strip(), None))
        tw.bind('<Double-Button-1>', wedit)
        f2 = ttk.Frame(self.insp); f2.pack(fill='x'); ttk.Button(f2,
text='Add wire…', command=self.add_wire_dialog).pack(side='left',
padx=2)
        ttk.Button(f2, text='Delete wire', command=lambda:
self.del_from_table(tw, 'wire')).pack(side='left', padx=2)
    def insp_cell(self, iid):
        kind, name = iid[0], iid[2:]; self.hdr('Library cell' if kd
== 'c' else 'Design module')
        sym = self.lib_symbol(iid); ttk.Label(self.insp, text=name,
font=(self.mono, 11, 'bold')).pack(anchor='w')
        if kind == 'c':
            c = self.lib.cells[name]; ttk.Label(self.insp,
text=describe_cell(c), style='Hint.TLabel').pack(anchor='w')
            ttk.Label(self.insp, text='Timing groups: NLDM %d · CCS %d
· ECSM %d    area %s' % (c.flags['nldm'], c.flags['ccs'],
c.flags['ecsm'], c.area), style='Hint.TLabel',
wraplength=300).pack(anchor='w')
        selsym_canvas(sym, 130)
        if kind == 'c':
            tv = self.make_table([('pin', 'Pin', 100), ('d', '', 36),
('f', 'Function', 190)], 8)
            for p in c.pins: tv.insert('', 'end', text=p.name +
('[%d:%d]' % (p.msb, p.lsb) if p.width > 1 else ''), values=('◀' if
p.dir == 'input' else '▶', p.func or ('3-state' if p.tri else '')))
        ttk.Button(self.insp, text='Place on sheet', command=lambda:
self.place_at_center(name)).pack(anchor='w', pady=6)
        ttk.Label(self.insp, text='or drag rom the list onto the
schematic', style='Hint.TLabel').pack(anchor='w')

    # ---------------- inspector edit callbacks ----------------
    def auto_declare(self, m, expr, width):
        mm = SIMPLE_SIG.match(expr)
        if not mm or mm.group(2): return
        base = mm.group(1)
        if base in m.nets or re.match(r'^\d', base): return
        add_net(m, base, (width - 1, 0) if width > 1 else None);
self.set_status('Declared new wire "%s"%s.' % (base, ' [%d:0]' %
(width - 1) if width > 1 else ''))
    def edit_pin(self, instname, pin, expr, width):
        m = self.cur_mod(); expr = expr.strip()
        def fn():
            inst = m.inst(instname); ensure_named(self.D, self.lib, inst)
            if expr == '': set_conn(m, inst, pin, '' if inst.prim else None)
            else: self.auto_declare(m, expr, width); set_conn(m, inst,
pin, expr)
            if inst.prim: inst.conns.sort(key=lambda c: 0 if c.pin ==
'Y' else int(c.pin[1:]))
        self.edit('connect %s.%s' % (instname, pin), [m.name], fn)
    def rename_inst(self, old, new):
        m = self.cur_mod()
        if not new or new == old: return
        if not (isId(new) or isEscId(new)) or m.inst(new) or new in
m.nets: messagebox.showwarning(APP_NAME, '"%s" is not a valid, unused
name in %s.' % (new, m.name)); self.refresh_inspector(); return
        def fn():
            i0 = m.inst(old); jinst(i0); ix_set(m, i0, False); i0.name
= new; ix_set(m, i0, True); soft_touch(m, i0)
            if 'i:' + old in m.layout: m.layout['i:' + new] =
m.layout.pop('i:' + old)
        if self.edit('rename ' + old, [m.name], fn): self.sel_node =
'i:' + new; self.refresh_inspector()
    def retype_inst(self, name, new):
        m = self.cur_mod(); inst = m.inst(name)
        if not new or new == inst.type: return
        if new == m.name: messagebox.showwarning(APP_NAME, 'A module
cannot instantiate itself.'); return
        rt = resolve_type(self.D, self.lib, new)
        def fn():
            jinst(inst)
            old = resolve_type(self.D, self.lib, inst.type); op =
type_pins(old, inst)
            if inst.positional and not inst.prim:
                for i, c in enumerate(inst.conns): c.pin = op[i].name
if i < len(op) else 'p%d' % i
                inst.positional = False
            names = {p.name for p in type_pins(rt, inst)}; dropped = []
            if rt[0] != 'unknown': inst.conns = [c for c in inst.conns
if c.pin in names or dropped.append(c.pin)]
            inst.type = new; inst.prim = rt[0] == 'prim'; soft_touch(m, inst)
            if dropped: self.set_status('Pins with no match in %s were
disconnected: %s' % (new, ', '.join(dropped)))
        self.edit('change type of ' + name, [m.name], fn)
    def affected_for(self, m): return [m.name] + [pm.name for pm, pi
in parents_of(self.D, m.name)]
    def rename_net(self, old, new):
        m = self.cur_mod()
        if not new or new == old: return
        if not (isId(new) or isEscId(new)) or new in m.nets or
m.inst(new): messagebox.showwarning(APP_NAME, '"%s" is not a valid,
unused name in %s.' % (new, m.name)); self.refresh_inspector(); return
        def fn():
            rename_net(self.D, m, old, new)
            if 'p:' + old in m.layout: m.layout['p:' + new] =
m.layout.pop('p:' + old)
        if self.edit('rename ' + old, self.affected_for(m), fn):
            if self.sel_sig == old: self.sel_sig = new
            self.refresh_inspector()
    def parse_range(self, s):
        s = s.strip().strip('[]')
        if not s or s == '1': return None
        mm = re.match(r'^(\d+)\s*:\s*(\d+)$', s)
        if mm: return (int(mm.group(1)), int(mm.group(2)))
        if s.isdigit(): return (int(s) - 1, 0)
        raise ValueError('Use a width like 7:0, or leave empty for one bit.')
    def set_shape(self, name, rng_s, dir_):
        m = self.cur_mod(); n = m.nets.get(name)
        try: rng = self.parse_range(rng_s) if dir_ is None else
((n.msb, n.lsb) if n.vec else None)
        except ValueError as ex: messagebox.showwarning(APP_NAME,
str(ex)); self.refresh_inspector(); return
        if dir_ is not None and dir_ not in ('input', 'output',
'inout'): messagebox.showwarning(APP_NAME, 'Direction must be input,
output or inout.'); self.refresh_inspector(); return
        self.edit('change %s' % name, self.affected_for(m), lambda:
set_net_shape(m, name, rng, dir_))
    def edit_assign(self, a, side, val):
        m = self.cur_mod()
        def fn(): setattr(a, side, val.strip()); touch(m);
[m.layout.pop(k) for k in [k for k in m.layout if k.startswith('a:')]]
        self.edit('edit assign', [m.name], fn)
    def rename_module_cb(self, new):
        m = self.cur_mod(); new = new.strip(); old = m.name
        if not new or new == old: return
        if not (isId(new) or isEscId(new)) or new in self.D.modules:
messagebox.showwarning(APP_NAME, 'Choose an unused, valid module
name.'); self.refresh_inspector(); return
        def fn():
            rename_module(self.D, old, new)
        if self.edit('rename module ' + old, self.affected_for(m) + [new], fn):
            self.mod = new
            if self.top == old: self.top = new
            if self.root_mod == old: self.root_mod = new
            self.path = [(i, new if mm == old else mm) for i, mm in
self.path]; self.refresh_all()
    def add_port_dialog(self):
        m = self.cur_mod(); r = FormDialog(self.root, 'Add port',
[('n', 'Name', '', None), ('d', 'Direction', 'input', ['input',
'output', 'inout']), ('w', 'Width (e.g. 7:0)', '', None)],
'Add').result
        if not r or not r['n'].strip(): return
        nm = r['n'].strip()
        if not (isId(nm) or isEscId(nm)) or nm in m.nets:
messagebox.showwarning(APP_NAME, 'Invalid or already used name.');
return
        try: rng = self.parse_range(r['w'])
        except ValueError as ex: messagebox.showwarning(APP_NAME,
str(ex)); return
        self.edit('add port ' + nm, self.affected_for(m), lambda:
add_port(self.D, m, nm, r['d'], rng))
    def add_wire_dialog(self):
        m = self.cur_mod(); r = FormDialog(self.root, 'Add wire',
[('n', 'Name', '', None), ('w', 'Width (e.g. 3:0)', '', None)],
'Add').result
        if not r or not r['n'].strip(): return
        nm = r['n'].strip()
        if not (isId(nm) or isEscId(nm)) or nm in m.nets:
messagebox.showwarning(APP_NAME, 'Invalid or already used name.');
return
        try: rng = self.parse_range(r['w'])
        except ValueError as ex: messagebox.showwarning(APP_NAME,
str(ex)); return
        self.edit('add wire ' + nm, [m.name], lambda: add_net(m, nm, rng, ''))
    def del_from_table(self, tv, kind):
        sel = tv.selection()
        if not sel or sel[0].startswith('x:'): return
        m = self.cur_mod(); name = sel[0][2:]
        if kind == 'port': self.edit('delete port ' + name,
self.affected_for(m), lambda: remove_port(self.D, m, name))
        else: self.edit('delete wire ' + name, [m.name], lambda:
remove_net(m, name))

    # ---------------- checks / problems ----------------
    def populate_problems(self):
        tv = self.prob; tv.delete(*tv.get_children()); self.prob_rows = {}
        allx = self.issues + self.vl_issues + self.lib_issues; cnt =
{'error': 0, 'warn': 0, 'info': 0}
        for i in allx: cnt[i.sev] += 1
        want = {'error': self.f_err.get(), 'warn': self.f_warn.get(),
'info': self.f_info.get()}; sf = self.src_filter.get(); rank =
{'error': 0, 'warn': 1, 'info': 2}; n = 0
        for i in sorted(allx, key=lambda x: rank[x.sev]):
            if not want[i.sev]: continue
            if sf.startswith('Same') and not (i.src == 'builtin' and
i.vl): continue
            if sf.startswith('Extra') and not (i.src == 'builtin' and
not i.vl and i.cat != 'library'): continue
            if sf.startswith('From') and i.src != 'verilator': continue
            if sf.startswith('Library') and i.cat != 'library': continue
            where = (i.mod + ('/' + i.inst if i.inst else '') + (' · '
+ i.net if i.net and not i.inst and i.src != 'verilator' else '')) if
i.mod else ('line %d' % i.line if i.line else '')
            iid = tv.insert('', 'end', text={'error': '✕', 'warn':
'▲', 'info': 'i'}[i.sev], values=(i.code, where, i.msg, 'verilator' if
i.src == 'verilaelse ('lint' if i.vl else 'extra')),
tags=(i.sev,))
            self.prob_rows[iid] = i; n += 1
            if n >= 1500: break
        self.prob_count.configure(text='%d errors · %d warnings · %d
notes' % (cnt['error'], cnt['warn'], cnt['info']))
    def on_problem_open(self, e):
        iid = self.prob.identify_row(e.y); i = self.prob_rows.get(iid)
        if i and i.mod and self.D and i.mod in self.D.modules:
self.goto(i.mod, i.inst, i.net)
    def goto(self, mod, inst='', net=''):
        if self.mod  mod: self.open_module(mod)
        v = self.get_view()
        if inst and 'i:' + inst in v.by_key:
            nd = v.by_key['i:' + inst]; self.select_node(nd.key);
self.center_on(nd.x + nd.sym.w / 2, nd.y + nd.sym.h / 2)
        elif net:
            s = v.sigs.get(net) or next((x for x in v.sig_list if
x.base == net), None)
            if s: self.select_sig(s.id)
    def center_on(self, wx, wy):
        W = self.canvas.winfo_width(); H = self.canvas.winfo_height();
self.vx = wx - W / self.z / 2; self.vy = wy - H / self.z / 2;
self.request_redraw()

    # ---------------- ECO log / source tab ----------------
    def refresh_eco_log(self):
        t = self.eco_text; t.configure(state='normal'); t.delete('1.0', 'end')
        t.insert('end', '\n'.join(self.D.eco_log) if self.D and
self.D.eco_log else 'No ECO connections yet.\nUse ECO wire (or drag
from pin to pin) to connect two pins anywhere in the hierarchy.')
        t.configure(state='disabled')
    def save_eco_log(self):
        if not self.D or not self.D.eco_log:
messagebox.showinfo(APP_NAME, 'The ECO log is empty.'); return
        p = filedialog.asksaveasfilename(defaultextension='.log',
initialfile='eco.log')
        if p: open(p, 'w').write('\n'.join(self.D.eco_log) + '\n')
    def refresh_source(self):
        t = self.src_text; t.delete('1.0', 'end'); m = self.cur_mod()
        if m: t.insert('end', (m.src + '\n') if (not m.dirty and
m.src) else module_text(m))
        self.src_msg.configure(text='')
    def apply_source(self):
        if not self.D: return
        d = parse_verilog(self.src_text.get('1.0', 'end'))
        if d.errors: self.src_msg.configure(text='Line %d: %s' %
d.errors[0], style='Err.TLabel'); return
        if not d.order: self.src_msg.configure(text='No module
found.', style='Err.TLabel'); return
        names = set(d.order)
        for n in list(names):
            if n in self.D.modules: names.update(pm.name for pm, pi in
parents_of(self.D, n))
        names.add(self.mod)
        def fn():
            for n in d.order:
                nm = d.modules[n]; old = self.D.modules.get(n);
nm.dirty = True; nm.lead = old.lead if old else '\n\n'; nm.layout =
old.layout if old else {}
                self.D.modules[n] = nm
                if not old: self.D.order.append(n)
        if self.edit('edit module source', sorted(names), fn):
self.src_msg.configure(text='Applied.', style='Hint.TLabel')

    # ---------------- Verilator ----------------
    def build_files(self, top):
        text, lmap, files = write_design(self.D); out = {'design.v': text}
        if self.lib: out['cells_sim.v'] = gen_cell_models(self.lib,
used_cell_names(self.D, self.lib))
        sh, cpp, readme = gen_verilator_files(self.D, self.lib, top, 64)
        out['run_verilator.sh'] = sh; out['tb_main.cpp'] = cpp;
out['README.txt'] = readme
        return out, lmap, files
    def run_verilator(self, sim):
        if not self.D: return
        exe = shutil.which('verilator')
        if not exe:
            messagebox.showinfo(APP_NAME, 'Verilator was not found in
PATH.\n\nUse File > Export Verilator bundle and run run_verilator.sh
where Verilator is installed, then use Tools > Import Verilator log to
bring the messages back.'); return
        top = self.vl_top.get() or self.top
        if not top: messagebox.showinfo(APP_NAME, 'Pick a top module
in the Verilator tab first.'); return
        out, lmap, files = self.build_files(top); tmp =
tempfile.mkdtemp(prefix='netlist_studio_')
        for n, t in out.items():
            with open(os.path.join(tmp, n), 'w') as f: f.write(t)
        self.last_export = lmap; self.bot.select(3);
self.vl_text.delete('1.0', 'end');
self.vl_status.configure(text='running Verilator…')
        def work():
            try:
                args = ['sh', os.path.join(tmp, 'run_verilator.sh'),
top] + (['--sim'] if sim else [])
                r = subprocess.run(args, capture_output=True,
text=True, timeout=900, cwd=tmp)
                self.vl_queue.put(('done', r.stdout + r.stderr,
r.returncode, tmp, lmap))
            except Exception as ex:
              self.vl_queue.put(('done', 'Could not run Verilator:
%s' % ex, -1, tmp, lmap))
        threading.Thread(target=work, daemon=True).start()
    def poll_queue(self):
        try:
            while True:
                kind, text, rc, tmp, lmap = self.vl_queue.get_nowait()
                self.vl_text.delete('1.0', 'end');
self.vl_text.insert('end', text);
self.vl_status.configure(text='finished (exit code %d). Files in %s' %
(rc, tmp))
                self.vl_issues =
self.map_vl(parse_verilator_log(text), lmap); self.flags_cache = None;
self.populate_problems(); self.request_redraw()
        except queue.Empty: pass
        self.root.after(150, self.poll_queue)
    def import_vl_log(self):
        p = filedialog.askopenfilename(title='Open
verilator_lint.log', filetypes=[('Log', '*.log *.txt'), ('All', '*')])
        if not p: return
        lmap = (self.last_export if self.last_export else
(write_design(self.D)[1] if self.D else {}))
        self.vl_issues =
self.map_vl(parse_verilator_log(read_text(p)), lmap);
self.populate_problems(); self.request_redraw()
        self.set_status('Imported %d Verilator messages.' % len(self.vl_issues))

    # ---------------- save / export ----------------
    def save_netlist(self, as_new=False):
        if not self.D: return
        p = self.save_path
        if as_new or not p:
            p = filedialog.asksaveasfilename(title='Save ECO netlist',
defaultextension='.v', initialfile=(os.path.splitext(self.v_name)[0] +
'_eco.v') if self.v_name else 'design_eco.v', filetypes=[('Verilog',
'*.v'), ('All', '*')])
            if not p: return
        try:
            with open(p, 'w') as f: lmap = write_design_to(self.D, f)
            self.last_export = lmap
            if self.D.eco_log:
                with open(os.path.splitext(p)[0] + '.eco.log', 'w') as
f: f.write('\n'.join(self.D.eco_log) + '\n')
        except OSError as ex: messagebox.showerror(APP_NAME, 'Could
not save: %s' % ex); return
        self.save_path = p; ed = sum(1 for m in
self.D.modules.values() if m.dirty)
        self.set_status('Saved %s: %d modules (%d edited, %d written
exactly as loaded)%s' % (p, len(self.D.order), ed, len(self.D.order) -
ed, ', plus ECO log' if self.D.eco_log else ''))
    def export_bundle(self):
        if not self.D: return
        top = self.vl_top.get() or self.top or self.D.order[0]; d =
filedialog.askdirectory(title='Choose a folder for the Verilator
bundle')
        if not d: return
        out, lmap, files = self.build_files(top); self.last_export = lmap
        res = self.issues if sum(len(m.insts) for m in
self.D.modules.values()) > AUTO_CHECK_MAX else run_checks(self.D,
self.lib, top)[0]; rep = ['Netlist Studio built-in checks', 'top: ' +
top, '']
        rep += ['[%s] %s  %s  %s' % (i.sev.upper(), i.code, (i.mod +
'/' + i.inst) if i.mod else '', i.msg) for i in res];
out['netlist_checks.txt'] = '\n'.join(rep) + '\n'
        os.makedirs(d, exist_ok=True)
        for n, t in out.items():
            with open(os.path.join(d, n), 'w') as f: f.write(t)
        os.chmod(os.path.join(d, 'run_verilator.sh'), 0o755)
        if self.D.eco_log: open(os.path.join(d, 'eco.log'),
'w').write('\n'.join(self.D.eco_log) + '\n')
        self.set_status('Bundle written to %s. Run:  sh
run_verilator.sh %s --sim' % (d, top))
    def export_svg(self):
        v = self.get_view()
        if not v: return
        p = filedialog.asksaveasfilename(defaultextension='.svg',
initialfile='schematic_%s.svg' % self.mod)
        if not p: return
        if len(v.nodes) > 6000 and not messagebox.askyesno(APP_NAME,
'This module has %d instances. The SVG will be very large. Continue?'
% len(v.nodes)): return
        bb = view_bounds(v); pad = 30; T = THEMES['light']; P =
SvgPainter(T, self.fam)
        paint_view(P, v, (bb[0] - pad, bb[1] - pad, bb[2] + pad, bb[3]
+ pad), {'z': 1.0, 'flags': {}, 'block': self.v_block.get(),
'detail_limit': 10 ** 9}, T)
        w = bb[2] - bb[0] + 2 * pad; h = bb[3] - bb[1] + 2 * pad
        open(p, 'w').write('<svg xmlns="http://www.w3.org/2000/svg"
viewBox="%.0f %.0f %.0f %.0f" width="%.0f" height="%.0f"><rect
x="%.0f" y="%.0f" width="%.0f" height="%.0f" fill="#ffffff"/>%s</svg>'
% (bb[0] - pad, bb[1] - pad, w, h, w, h, bb[0] - pad, bb[1] - pad, w,
h, ''.join(P.out)))
        self.set_status('Schematic saved to ' + p)

    # ---------------- dialogs ----------------
    def show_help(self):
        t = tk.Toplevel(self.root); t.title('Shortcuts and ECO
workflow'); t.geometry('+%d+%d' % (self.root.winfo_rootx() + 160,
self.root.winfo_rooty() + 80))
        txt = tk.Text(t, width=86, height=30, wrap='word',
font=(self.fam, 10), padx=12, pady=10); txt.pack()
        txt.insert('end', HELP_TEXT); txt.configure(state='disabled',
bg=self.T['fill'], fg=self.T['ink'])

    # ================= background jobs (parse, check, search) =================
    def run_job(self, title, fn, done):
        if self.job: messagebox.showinfo(APP_NAME, 'Another task is
still running.'); return
        top = tk.Toplevel(self.root); top.title(APP_NAME);
top.transient(self.root); top.resizable(False, False)
        f = ttk.Frame(top, padding=16); f.pack()
        ttk.Label(f, text=title, font=(self.fam, 11, 'bold')).pack(anchor='w')
        pb = ttk.Progressbar(f, length=460, mode='determinate',
maximum=1000); pb.pack(pady=10)
        sub = ttk.Label(f, text='starting…', style='Hint.TLabel',
width=70); sub.pack(anchor='w')
        job = {'cancel': False, 'frac': 0.0, 'msg': '', 'res': None,
'err': None, 'tb': '', 'done': False, 'top': top, 'pb': pb, 'sub':
sub, 'cb': done, 't0': time.time()}
       ef abort():
            """stop the task now: flag it, raise JobCancelled inside
the worker thread and release the GUI at once"""
            job['cancel'] = True; job['aborted'] = True
            tid = job.get('tid')
            if tid:
                try:
                    import ctypes;
ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_ulong(tid),
ctypes.py_object(JobCancelled))
                except Exception: pass
        bf = ttk.Frame(f); bf.pack(anchor='e', pady=(8, 0))
        ttk.Button(bf, text='Exit tool',
command=self.exit_now).pack(side='right', padx=(6, 0))
        ttk.Button(bf, text='Cancel', command=abort).pack(side='right')
        top.protocol('WM_DELETE_WINDOW', abort); top.update_idletasks()
        top.geometry('+%d+%d' % (self.root.winfo_rootx() + 260,
self.root.winfo_rooty() + 200))
        try: top.wait_visibility(); top.grab_set()
        except tk.TclError: pass
        def prog(frac, msg=''):
            job['frac'] = frac
            if msg: job['msg'] = msg
        def work():
            job['tid'] = threading.get_ident()
            try: job['res'] = fn(prog, lambda: job['cancel'])
            except JobCancelled: job['err'] = 'cancelled'
            except BaseException as ex: job['err'] = ex; job['tb'] =
traceback.format_exc()
            if job.get('aborted'): job['res'] = None; job['err'] = 'cancelled'
            job['done'] = True
        self.job = job; threading.Thread(target=work,
daemon=True).start(); self.root.after(60, self.poll_job)
    def exit_now(self):
        """leave the tool immediately, also in the middle of a task"""
        try:
            if self.job: self.job['cancel'] = True; self.job['aborted'] = True
            self.root.destroy()
        except Exception: pass
        os._exit(0)
    def poll_job(self):
        job = self.job
        if job is None: return
        if job.get('aborted'): job['done'] = True; job['err'] = 'cancelled'
        job['pb']['value'] = int(1000 * min(1.0, job['frac']))
        job['sub'].configure(text='%s   ·   %.0f ss' % (job['msg'] or
'working…', time.time() - job['t0'], ('   ·   peak memory %d MB' %
mem_mb()) if mem_mb() else ''))
        if not job['done']: self.root.after(80, self.poll_job); return
        self.job = None
        try: job['top'].grab_release()
        except tk.TclError: pass
        job['top'].destroy(); err = job['err']
        if err == 'cancelled':
            self.set_status('Cancelled.')
            try: trim_memory()
            except Exception: pass
            return
        if err is  None:
            if isinstance(err, LibCheckError):
                self.set_status('Netlist rejected: cells missing from
the libraries.'); self.con_write('Error: ' + err.report) if
hasattr(self, 'con_text') else None
                messagebox.showerror(APP_NAME + ' - library check',
err.report, parent=self.root); return
            log_crash(job['tb'])
            if isinstance(err, MemoryError):
messagebox.showerror(APP_NAME, 'Ran out of memory (peak %d
MB).\\n\\nThe netlist is larger than this machine can hold. Close
other programs, or split the netlist.' % mem_mb())
            else: messagebox.showerror(APP_NAME, 'The task
failed:\\n%s\\n\\nDetails were written to %s' % (err, CRASH_LOG))
            return
        job['cb'](job['res'])

    # ================= loading =================
    def load_lib_file(self, path): self.load_lib_files([path])
    def load_netlist_file(self, path): self.load_netlist_files([path])
    def load_lib_files(self, paths, then=None):
        paths = [p for p in paths if p]; n = len(paths)
        if self.D is not None:
            messagebox.showwarning(APP_NAME, 'Libraries are locked
while a netlist is loaded (cells were resolved against the libraries
read before it).\n\nUse File > Close design and libraries, then read
all libraries again (select several files at once, or open them one
after another) before the netlist.'); return
        label = os.path.basename(paths[0]) if n == 1 else '%s + %d
more' % (os.path.basename(paths[0]), n - 1)
        def work(prog, cancel):
            lib = None; notes = []
            if n >= 2 and fork_ok():                    # several
libraries: parse them on separate cores
                prog(0.05, 'parsing %d libraries on %d cores' % (n,
min(n, n_workers())))
                try: parsed = fork_map(lambda i:
parse_liberty(read_text(paths[i])), n)
                except Exception: parsed = None
                if parsed is not None:
                    for k, one in enumerate(parsed):
                        if lib is None: lib = one
                        else: merge_lib(lib, one,
os.path.basename(paths[k]), notes)
                    return lib, notes
            for k, path in enumerate(paths):
                if cancel(): raise JobCancelled()
                nm = os.path.basename(path); f0 = k / n; f1 = (k + 1) / n
                prog(f0 + 0.05 / n, 'reading %s (%d/%d)' % (nm, k + 1,
n)); text = read_text(path)
                prog(f0 + 0.3 / n, 'parsing %s (%d/%d)' % (nm, k + 1,
n)); one = parse_liberty(text); del text
                if lib is None: lib = one
                else: merge_lib(lib, one, nm, notes)
                del one
            return lib, notes
        def done(res):
            lib, notes = res
            names = [os.path.basename(p) for p in paths]
            if self.lib is not None and self.lib_files:        #
append to the libraries already read
                add_notes = []; merge_lib(self.lib, lib, label,
add_notes); notes = list(self.lib_notes) + notes + add_notes; lib =
self.lib
            self.lib_files = self.lib_files + [x for x in names if x
not in self.lib_files]; self.lib_notes = notes
            self.lib = lib; self.lib_name = self.lib_files[0] if
len(self.lib_files) == 1 else '%s + %d more' % (self.lib_files[0],
len(self.lib_files) - 1)
            self.invalidate_all(); self.populate_library();
self.update_lib_info()
            self.lib_issues = [Issue('warn', 'LIBDUP', 'syntax', t)
for t in notes]
            self.populate_problems()
            st = lib.stats
            self.set_status('Liberty loaded from %d file(s): %d cells
(NLDM %d, CCS %d, ECSM %d)%s.' % (n, st['cells'], st['nldm'],
st['ccs'], st['ecsm'], ('; %d note(s), see Problems' % len(notes)) if
notes else ''))
            self.run_checks_now(); self.request_redraw()
            if then: self.root.after(50, then)
        self.run_job('Loading Liberty %s' % label, work, done)
    def load_netlist_files(self, paths):
        paths = [p for p in paths if p]; n = len(paths)
        label = os.path.basename(paths[0]) if n == 1 else '%s + %d
more' % (os.path.basename(paths[0]), n - 1)
        def work(prog, cancel):
            D = None; notes = []
            for k, path in enumerate(paths):
                if cancel(): raise JobCancelled()
                nm = os.path.basename(path); f0 = k / n
                prog(f0, 'reading %s (%d/%d)' % (nm, k + 1, n)); text
= read_text(path)
                one = parse_verilog(text, lambda f, msg='': prog(f0 +
f / n, '%s (%d/%d) %s' % (nm, k + 1, n, msg)), cancel); del text; text
= None
                for m in one.modules.values(): m.srcfile = nm
                if D is None: D = one
                else: merge_design(D, one, nm, notes)
                del one
            D.notes = notes; D.files = [os.path.basename(p) for p in paths]
            mode = self.lib_check
            if mode != 'off':
                prog(0.99, 'checking leaf cells against the libraries')
                used, missing, nofp = collect_leaf_cells(D, self.lib)
                rep = lib_check_report(D, self.lib, used, missing, nofp)
                if missing and mode == 'strict':
                    del D; raise LibCheckError('Netlist %s was NOT
loaded.\n\n%s\n\nRead the libraries that define these cells (before
the netlist), or switch Tools > Library check to "warn" / "off".' %
(label, rep))
                D.lib_report = rep
            return D
        def done(D):
            if not D.order: messagebox.showerror(APP_NAME, 'No module
found in %s.' % label); return
            self.install_design(D, label)
            if getattr(D, 'lib_report', ''):
                try: self.con_write('Library check:\n' + D.lib_report)
                except Exception: pass
            self.save_path = os.path.splitext(paths[0])[0] +
('_merged' if n > 1 else '') + '_eco.v'
            tot = sum(len(m.insts) for m in D.modules.values())
            self.set_status('Netlist loaded from %d file(s): %d
modules, %d instances%s%s.' % (n, len(D.order), tot, (', %d parse
problems' % len(D.errors)) if D.errors else '', ('; %d duplicate
module(s) ignored' % len(getattr(D, 'dups', []))) if getattr(D,
'dups', None) else ''))
        self.run_job('Loading netlist %s' % label, work, done)

    # ================= checks =================
    def run_checks_now(self, manual=False):
        if not self.D: return
        tot = sum(len(m.insts) for m in self.D.modules.values())
        def finish(res):
            self.issues, t = res; self.flags_cache = None;
self.populate_problems(); self.request_redraw()
            if manual: self.set_status('Checks finished: %d findings.'
% len(self.issues))
        if tot <= 4000: finish(run_checks(self.D, self.lib, self.top)); return
        if tot > AUTO_CHECK_MAX and not manual:
            self.issues = [Issue('info', 'SKIPPED', 'syntax', 'Large
design (%d instances): automatic checks are off. Press "Checks" to run
them with progress.' % tot)]
            self.flags_cache = None; self.populate_problems(); return
        self.run_job('Running structural and logic checks', lambda
prog, cancel: run_checks(self.D, self.lib, self.top, prog, cancel),
finish)
    def schedule_check(self):
        if not self.v_auto.get(): return
        if sum(len(m.insts) for m in self.D.modules.values()) <= 4000:
self.run_checks_now(); return
        if self.chk_job: self.root.after_cancel(self.chk_job)
        self.chk_job = self.root.after(800, lambda: self.run_checks_now(False))

    # ================= view =================
    def get_view(self):
        m = self.cur_mod()
        if m is None: return None
        block = self.v_block.get(); win = page_window(m, getattr(m,
'_page', 0)) if is_paged(m) else None
        if m._view is None or m._view.block != block or m._view.window != win:
            if len(m.insts) > 20000 and win is None:
self.set_status('Building view of %d instances…' % len(m.insts));
self.root.update_idletasks()
            v = build_view(self.D, self.lib, m, block, win);
apply_layout(v); m._view = v; v.label_mode = None
        v = m._view
        lm = self.v_wires.get() == 'labels' or (self.v_wires.get() ==
'auto' d len(v.sig_list) > 1500)
        if v.label_mode != lm: layout_sigs(v, lm)
        return v
    # ---- paging of huge modules ----
    def paged(self):
        m = self.cur_mod(); return bool(m) and is_paged(m)
    def set_page(self, k, fit=True):
        m = self.cur_mod()
        if m is None or not is_paged(m): return
        k = max(0, min(int(k), n_pages(m) - 1))
        if k == getattr(m, '_page', 0) and m._view is not None and
m._view.window is not None: return
        m._page = k; m._view = None; self.sel_node = self.sel_sig =
None; self.need_fit = fit
        self.refresh_crumbs(); self.refresh_inspector(); self.request_redraw()
    def page_of_index(self, idx): return idx // PAGE_SIZE
    def show_index(self, idx, fit=False):
        m = self.cur_mod()
        if m is not None and is_paged(m):
self.set_page(self.page_of_index(idx), fit)
    def ensure_page_for(self, f):
        """make sure the page of the current module that holds object
f is displayed"""
        m = self.cur_mod()
        if m is None or not is_paged(m): return
        v = m._view
        key = ('i:' + f.name) if f.kind in ('inst', 'pin') else ('p:'
+ f.name) if f.kind in ('port', 'topport') else None
        if key and v is not None and key in v.by_key: return
        if f.kind in ('inst', 'pin'):
            for k, i in enumerate(m.insts):
                if i.name == f.name: self.show_index(k); return
        elif f.kind == 'net':
            rx = re.compile(r'(?<![\w$])' + re.escape(f.name) +
r'(?![\w$])'); self.set_status('Looking for the first user of %s…' %
f.name); self.root.update_idletasks()
            for k, i in enumerate(m.insts):
                for c in i.conns:
                    if f.name in c.expr and rx.search(c.expr):
self.show_index(k); return
    def redraw(self):
        self.redraw_pending = False
        c = self.canvas; c.delete('all'); v = self.get_view()
        W = max(c.winfo_width(), 100); H = max(c.winfo_height(), 100)
        if v is None:
            c.create_text(W / 2, H / 2, text='Open a netli (and its
Liberty file) to start.\nOr press "Sample" for a demo design.',
fill=self.T['mute'], font=(self.fam, 13), justify='center'); return
        if self.need_fit: self.need_fit = False; self.fit_now(v, W, H)
        z = self.z; vis = (self.vx, self.vy, self.vx + W / z, self.vy + H / z)
        if self.v_grid.get(): self.draw_grid(W, H)
        P = TkPainter(c, self.vx, self.vy, z, self.T, self.fam);
groups, ms, ovr = self.hl_state(v)
        st = {'z': z, 'sel_node': self.sel_node, 'sel_sig':
self.sel_sig, 'flags': self.issue_flags(), 'net_names':
self.v_nets.get(), 'pin_names': self.v_pins.get(),
              'tags': True, 'block': self.v_block.get(), 'pending':
self.pending_local(), 'hl': groups, 'msel': ms, 'ovr': ovr,
'detail_limit': 1400}
        paint_view(P, v, vis, st, self.T)
        if v.big and v.overview:
            c.create_rectangle(0, 0, W, 22, fill=self.T['panel'],
outline=''); c.create_text(8, 11, anchor='w', fill=self.T['ink'],
font=(self.fam, -12),
                text='Overview of %d cells. Zoom in (mouse wheel) to
see symbols and wires. Use Find to locate objects.' % len(v.nodes))
        if self.wire_drag: self.draw_rubber()
    def fit_now(self, v, W, H):
        bb = view_bounds(v); pad = 40; w = bb[2] - bb[0] + 2 * pad; h
= bb[3] - bb[1] + 2 * pad
        self.z = max(0.003, min(W / w, H / h, 1.6)); self.vx = bb[0] -
pad - (W / self.z - w) / 2; self.vy = bb[1] - pad - (H / self.z - h) /
2
    def zoom_at(self, f, sx=None, sy=None):
        W = self.canvas.winfo_width(); H = self.canvas.winfo_height()
        if sx is None: sx, sy = W / 2, H / 2
        wx = self.vx + sx / self.z; wy = self.vy + sy / self.z; nz =
max(0.003, min(6.0, self.z * f))
        self.z = nz; self.vx = wx - sx / nz; self.vy = wy - sy / nz;
self.request_redraw()
    def pending_local(self):
        hp = self.pending
        if isinstance(hp, HPin) and tuple(hp.path) == self.cur_path():
return ('i:' + hp.inst if hp.inst else 'p:' + hp.pin, hp.pin if
hp.inst else 'p')
        return None
    def on_drag(self, e):
        if not self.press: return
        p = self.press
        if abs(e.x - p['x']) + abs(e.y - p['y']) > 4: p['moved'] = True
        if getattr(self, 'band', None) and p['moved'] and not self.pan:
            b = self.band; self.canvas.delete('band')
            self.canvas.create_rectangle(b['x'], b['y'], e.x, e.y,
outline=self.T['acc'], dash=(4, 3), width=1, tags='band'); return
        if self.pan and p['hit'] is None:
            v0 = self.get_view()
            if v0 is not None and v0.big and v0.overview and
p['moved'] and not p.get('hinted'):
                p['hinted'] = True; self.set_status('Overview: zoom in
(mouse wheel or double-click) to move individual cells.')
            self.pan_move(e); return
        h = p['hit']
        if self.node_drag and p['moved']:
            nd = self.node_drag['nd']; d = self.node_drag; wx, wy =
self.to_world(e.x, e.y); v = self.get_view()
            nx = d['ox'] + wx - d['wx']; ny = d['oy'] + wy - d['wy']
            if self.v_snap.get(): nx, ny = snap_to(nx), snap_to(ny)
            dx = (nx - nd.x) * self.z; dy = (ny - nd.y) * self.z
            if dx or dy:
                ox, oy = nd.x, nd.y; nd.x, nd.y = nx, ny;
self.cur_mod().layout[nd.key] = (nx, ny)
                if v.big: reindex_node(v, nd, ox, oy)
                self.canvas.move('n%d' % nd.idx, dx, dy);
self.repaint_node_wires(nd)
            return
        if h and h[0] == 'pin' and p['moved']:
            self.wire_drag = {'node': h[1], 'pin': h[2], 'x': e.x,
'y': e.y}; self.canvas.delete('rubber'); self.draw_rubber()
    def repaint_node_wires(self, nd):
        v = self.get_view(); done = set(); P = TkPainter(self.canvas,
self.vx, self.vy, self.z, self.T, self.fam)
        st = {'z': self.z, 'tags': True, 'net_names': self.v_nets.get()}
        for ids in nd.pin_sigs.values():
            for sid in ids:
                s = v.sigs.get(sid)
                if s is None or sid in done: continue
                done.add(sid); self.canvas.delete('s%d' % s.idx);
sig_geom(s, v.label_mode)
                paint_sig(P, s, st, self.T, set([sid]), self.sel_sig,
None, self.z > 0.3)

    # ================= mouse: ECO with pins and wires =================
    def on_release(self, e):
        p = self.press; self.press = None; self.pan = None; band =
getattr(self, 'band', None); self.band = None
        if not p: return
        v = self.get_view()
        if p.get('panonly'): return
        if band is not None:
            self.canvas.delete('band')
            if p['moved']: self.band_select(band, e, p['ctrl'] or
p['shift']); return
        if p.get('ctrl') and p['hit'] is not None and p['hit'][0] in
('node', 'pin', 'sig'):
            self.toggle_hit(p['hit']); return
        if self.node_drag:
            moved = p['moved']; self.node_drag = None
            if moved: self.request_redraw()
            return
        h = p['hit']
        if self.wire_drag:
            src = self.wire_drag; self.wire_drag = None;
self.canvas.delete('rubber')
            wx, wy = self.to_world(e.x, e.y); t = hit_test(v, wx, wy,
self.z); a = self.hpin_of(src['node'], src['pin'])
            if a is None: self.set_status('Assign statements cannot be
connected from the canvas; edit them in the Verilog tab.');
self.request_redraw(); return
            if t and t[0] == 'pin' and not (t[1] is src['node'] and
t[2] == src['pin']):
                b = self.hpin_of(t[1], t[2])
                if b: self.do_eco_connect(a, b)
                else: self.request_redraw()
            elif t and t[0] == 'sig' and not t[1].stub:
self.do_eco_connect(a, HNet(self.cur_path(), t[1].id))
            else: self.request_redraw()
            return
        if h is None:
            if not p['moved'] and not p.get('ctrl'): self.select_none()
            return
        wire_mode = self.mode.get() == 'wire' or p['shift']
        if h[0] == 'pin':
            nd, pin = h[1], h[2]
            if wire_mode: self.eco_click(nd, pin)
            else:
                sids = nd.pin_sigs.get(pin)
                if sids: self.select_sig(sids[0])
                else: self.select_node(nd.key)
                if nd.kind == 'inst': self.sel_pin = (nd.ref.name, pin)
        elif h[0] == 'sig':
            if wire_mode and not h[1].stub: self.eco_click_sig(h[1])
            else: self.select_sig(h[1].id)
        elif h[0] == 'node' and self.mode.get() == 'wire':
self.select_node(h[1].key)
    def eco_click_sig(self, s):
        hn = HNet(self.cur_path(), s.id)
        if self.pending is None: self.set_pending(hn); self.sel_sig =
s.id; self.sel_node = None; self.refresh_inspector()
        elif isinstance(self.pending, HNet): self.set_status('Pick a
pin to connect to the wire %s.' % hstr(self.pending))
        else: self.complete_pending(hn)
    def eco_click(self, nd, pin):
        hp = self.hpin_of(nd, pin)
        if hp is None: self.set_status('Assign statements cannot be
connected from the canvas.'); return
        if self.pending is None: self.set_pending(hp)
        else: self.complete_pending(hp)
    def update_pending_status(self):
        if self.pending: self.set_status('ECO: from %s  ->  click the
destination (pin, or wire to reuse it). You may navigate to any module
first. Esc cancels.' % hstr(self.pending))
        elif self.mode.get() == 'wire': self.set_status('ECO wire
mode: click a pin or a wire, then the pin it should connect to.
Dragging a pin onto a pin or a wire also works.')
    def on_context(self, e):
        if not self.D: return
        v = self.get_view(); wx, wy = self.to_world(e.x, e.y); h =
hit_test(v, wx, wy, self.z); m = tk.Menu(self.root, tearoff=0)
        sel_net = HNet(self.cur_path(), self.sel_sig) if self.sel_sig
and self.sel_sig in v.sigs and not v.sigs[self.sel_sig].stub else None
        if h and h[0] == 'pin':
            nd, pin = h[1], h[2]; hp = self.hpin_of(nd, pin)
            if hp:
                m.add_command(label='ECO: start connection from %s' %
hpin_str(hp), command=lambda: self.set_pending(hp))
                if self.pending: m.add_command(label='ECO: connect
pending %s here' % ('wire' if isinstance(self.pending, HNet) else
'pin'), command=lambda: self.complete_pending(hp))
                if sel_net and nd.kind == 'inst':
m.add_command(label='ECO: connect this pin to selected wire %s
(reuse)' % sel_net.expr, command=lambda: self.do_eco_connect(hp,
sel_net))
                if nd.kind == 'inst':
                    m.add_separator()
                    sg = (nd.pin_sigs.get(pin) or [None])[0]; pdir =
nd.sym.pins[pin]['dir']
                    if sg and sg in v.sigs and not v.sigs[sg].stub:
                        m.add_command(label='ECO: insert repeater
chain on net %s…' % sg, command=lambda: self.repeater_dialog(sg,
(nd.ref.name, pin) if pdir != 'out' else None))
                        m.add_command(label='ECO: remove repeater
chain after net %s' % sg, command=lambda:
self.gui_cmd('eco_delete_repeater', ['-net', '/'.join(self.cur_path()
+ (sg,))]))
                    self.eco_menu_for_inst(m, nd); m.add_separator()
                    m.add_command(label='Disconnect pin',
command=lambda: self.eco_simple('disconnect', h)
                    m.add_command(label="Tie to 1'b0", command=lambda:
self.eco_simple('tie0', hp)); m.add_command(label="Tie to 1'b1",
command=lambda: self.eco_simple('tie1', hp))
        elif h and h[0] == 'sig' and not h[1].stub:
            pre_sel = self.selection_found(); hn =
HNet(self.cur_path(), h[1].id); self.select_sig(h[1].id)
            m.add_command(label='ECO: start connection from wire %s' %
h[1].id, command=lambda: self.set_pending(hn))
            if self.pending and not isinstance(self.pending, HNet):
m.add_command(label='ECO: connect pending pin to this wire (reuse)',
command=lambda: self.complete_pending(hn))
            m.add_separator(); sid = h[1].id
            m.add_command(label='ECO: insert repeater chain on %s…' %
sid, command=lambda: self.repeater_dialog(sid, sel=pre_sel))
            m.add_command(label='ECO: remove repeater chain after %s'
% sid, command=lambda: self.gui_cmd('eco_delete_repeater', ['-net',
'/'.join(self.cur_path() + (sid,))]))
        elif h and h[0] == 'ne':
            self.select_node(h[1].key); nd = h[1]
            if nd.kind == 'inst':
                if nd.rt[0] == 'module': m.add_command(label='Go
inside %s' % nd.ref.type, command=lambda:
self.open_module(nd.ref.type, self.path + [(nd.ref.name,
nd.ref.type)], self.root_mod))
                m.add_command(label='Duplicate',
command=self.duplicate_selected)
                if nd.rt[0] == 'cell': self.eco_menu_for_inst(m, nd)
            m.add_command(label='Delete', command=self.delete_selection)
        # ---- every ECO action is reachable from any right-click ----
        if m.index('end') is not None: m.add_separator()
        cur = self.cur_path()
        def hn(name): return '/'.join(cur + (name,))
        inst_here = h[1].ref.name if (h and h[0] in ('pin', 'node')
and h[1].kind == 'inst') else None
        net_here = (h[1].id if (h and h[0] == 'sig' and not h[1].stub)
else None) or (self.sel_sig if (self.sel_sig and self.sel_sig in
v.sigs and not v.sigs[self.sel_sig].stub) else None)
        eco = tk.Menu(m, tearoff=0); m.add_cascade(label='ECO
commands', menu=eco)
        eco.add_command(label='Add instance here…', command=lambda:
self.add_instance_dialog(wx, wy))
        eco.add_command(label='Add port…', command=self.add_port_dialog)
        eco.add_command(label='Add wire…', command=self.add_wire_dialog)
        eco.add_separator()
        if inst_here:
            eco.add_command(label='Delete instance %s' % inst_here,
command=lambda: self.gui_cmd('delete_inst', [hn(inst_here)]))
            eco.add_command(label='Clone instance %s' % inst_here,
command=lambda: self.gui_cmd('clone_inst', ['-inst', hn(inst_here),
'-num_clones', '1']))
            eco.add_command(label='Clone with inputs tied to 0',
command=lambda: self.gui_cmd('clone_inst', ['-inst', hn(inst_here),
'-num_clones', '1', '-auto_tie', '1']))
            eco.add_command(label='Clone by count…', command=lambda:
self.clone_dialog([hn(inst_here)]))
            nd_here = h[1]
            for sg in self.out_nets_of(nd_here)4]:
                eco.add_command(label='eco_add_repeater on output net
%s…' % sg, command=lambda sg=sg: self.repeater_dialog(sg))
                eco.add_command(label='eco_delete_repeater after
output net %s' % sg, command=lambda sg=sg:
self.gui_cmd('eco_delete_repeater', ['-net', hn(sg)]))
            eco.add_separator()
        if net_here:
            eco.add_command(label='Insert repeater chain on %s…' %
net_here, command=lambda: self.repeater_dialog(net_here))
            eco.add_command(label=ove repeater chain after %s' %
net_here, command=lambda: self.gui_cmd('eco_delete_repeater', ['-net',
hn(net_here)]))
            eco.add_command(label='Delete wire %s' % net_here,
command=lambda: self.gui_cmd('delete_net', [hn(net_here)]))
            eco.add_separator()
        if h and h[0] == 'node' and h[1].kind == 'port':
            pn = h[1].ref.name if hasattr(h[1].ref, 'name') else None
            if pn and not self.path: eco.add_command(label='Delete
port %s' % pn, command=lambda: self.gui_cmd('delete_ports', [pn]))
        if self.msel:
            names = [found_str(f).lstrip('/') for f in self.msel if
f.kind == 'inst']
            if names:
                eco.add_command(label='Delete %d selected instance(s)'
% len(names), command=lambda: self.gui_cmd('delete_inst', [names]))
                eco.add_command(label='Clone %d selected instance(s)'
% len(names), command=lambda: self.gui_cmd('clone_inst', ['-inst',
names, '-num_clones', '1']))
                eco.add_command(label='Clone %d selected instance(s)
by count…' % len(names), command=lambda: self.clone_dialog(names))
            nets = [f for f in self.msel if f.kind == 'net']
            for f in nets[:1]:
                nm = found_str(f).lstrip('/')
                eco.add_command(label='eco_add_repeater on selected
net %s…' % f.name, command=lambda f=f:
(self.repeater_dialog('/'.join(f.path + (f.name,)), absolute=True)))
            if nets:
                eco.add_command(label='eco_delete_repeater after %d
selected net(s)' % len(n, command=lambda:
[self.gui_cmd('eco_delete_repeater', ['-net',
found_str(f).lstrip('/')]) for f in nets])
        eco.add_command(label='eco_add_repeater on net…',
command=self.repeater_net_dialog)
        eco.add_command(label='eco_delete_repeater on net…',
command=lambda: (lambda n: n and self.gui_cmd('eco_delete_repeater',
['-net', n]))(self.net_prompt('Remove repeater chain: which net?',
self.sel_sig or '')))
        eco.add_separator()
        eco.add_command(label='Deselect all', command=lambda:
f.shell.run('deselect', ['-all']), self.select_none()))
        eco.add_separator()
        eco.add_command(label='Undo', state='normal' if self.undo else
'disabled', command=self.do_undo)
        eco.add_command(label='Redo', state='normal' if self.redo else
'disabled', command=self.do_redo)
        eco.add_command(label='Open ECO console (Ctrl+K)',
command=self.focus_console)
        m.add_command(label='Add instance here…', command=lambda:
self.add_instance_dialog(wx, wy)); m.add_command(label='Fit to
ndow', command=self.fit_view)
        m.tk_popup(e.x_root, e.y_root)
    def do_eco_connect(self, a, b):
        D = self.D; lib = self.lib; root = self.root_mod
        try: names = affected_modules(D, root, a, b)
        except EcoError as ex: messagebox.showwarning(APP_NAME,
str(ex)); self.request_redraw(); return
        res = self.edit('ECO connect', names, lambda: eco_connect(D,
lib, root, a, b, ask=self.ask_uniquify, style=self.eco_style.get(),
ask_bit=self.ask_bit))
        if res:
            log, created = res; newn = sum(1 for l in log if 'new net' in l)
            self.set_status('Connected %s -> %s  (%d change%s, %s)' %
(hstr(a), hstr(b), len(log) - 1, '' if len(log) == 2 else 's', 'new
net created' if newn else 'no new net needed'))
            self.refresh_eco_log(); v = self.get_view()
            if v is not None and not isinstance(b, HNet) and
tuple(b.path) == self.cur_path() and b.inst:
                nd = v.by_key.get('i:' + b.inst)
                if nd and nd.pin_sigs.get(b.pin): self.sel_sig =
nd.pin_sigs[b.pin][0]; self.sel_node = None; self.refresh_inspector();
self.request_redraw()
        else: self.request_redraw()

    # ================= undo snapshots (fast, compact) =================
    def snapshot(self, names):
        D = self.D; mods = {}
        for n in names:
            m = D.modules.get(n)
            if m is None: mods[n] = None; continue
            if len(m.insts) > JOURNAL_MIN: mods[n] = BIGMARK; continue
            try: mods[n] = pickle.dumps(m, protocol=pickle.HIGHEST_PROTOCOL)
            except Exception: mods[n] = copy.deepcopy(m)
        return {'mods': mods, 'order': list(D.order), 'ni':
D.eco_net_idx, 'pi': D.eco_port_idx, 'log': list(D.eco_log)}
    def restore(self, sn):
        D = self.D
        for n, b in sn['mods'].items():
            if b is BIGMARK: continue
            if b is None: D.modules.pop(n, None)
            else: D.modules[n] = pickle.loads(b) if isinstance(b,
(bytes, bytearray)) else copy.deepcopy(b)
        D.order = list(sn['order']); D.eco_net_idx = sn['ni'];
D.eco_port_idx = sn['pi']; D.eco_log = list(sn['log'])
        if self.mod not in D.modules: self.mod = D.order[0]; self.path
= []; self.root_mod = self.mod

    # ================= hierarchy tree (built lazily) =================
    def populate_hier(self):
        tv = self.hier; tv.delete(*tv.get_children()); self.hier_map =
{}; D = self.D
        if not D: return
        self.memo_cells = {}; self.memo_kids = {}
        tops = find_tops(D) or D.order[:1]
        roots = [self.hier_add('', t, '', [], t, True) for t in tops]
        self.hier_open_levels(roots, HIER_EAGER_NODES)
        shown = set()
        def mark(mn, guard=()):
            if mn in shown or mn in guard: return
            shown.add(mn)
            for t in sorted({i.type for i in child_insts(D,
D.modules[mn])}): mark(t, guard + (mn,))
        for t in tops: mark(t)
        for mn in D.order:
            if mn not in shown: self.hier_add('', mn, '', [], mn,
False); mark(mn)
        self.sync_hier()
    def cells_in(self, mn):
        memo = self.memo_cells
        if mn in memo: return memo[mn]
        memo[mn] = 0; n = 0; D = self.D
        for t, c in type_counts(D.modules[mn]).items(): n += c *
(self.cells_in(t) if t in D.modules else 1)
        memo[mn] = n; return n
    def has_kids(self, mn):
        r = self.memo_kids.get(mn)
        if r is None: r = self.memo_kids[mn] = any(t in self.D.modules
for t in inst_type_set(self.D.modules[mn]))
        return r
    def hier_add(self, parent, mn, inst, steps, root, expand):
        tv = self.hier; m = self.D.modules[mn]
        iid = tv.insert(parent, 'end', text=(('%s  (%s)' % (inst, mn))
if inst else mn) + ('  ●' if m.dirty else ''),
values=(self.cells_in(mn),), open=False)
        self.hier_map[iid] = (mn, list(steps), root)
        if self.has_kids(mn):
            tv.insert(iid, 'end', iid=iid + '#lazy', text='…', values=('',))
            if expand: self.hier_fill(iid); tv.item(iid, open=True)
        return iid
    def hier_fill(self, iid):
        if nelf.hier.exists(iid + '#lazy'): return
        self.hier.delete(iid + '#lazy'); mn, steps, root =
self.hier_map[iid]; n = 0
        for i in child_insts(self.D, self.D.modules[mn]):
            if len(steps) < 40 and i.type not in [root] + [t for _, t
in steps] + [mn]:
                self.hier_add(iid, i.type, i.name, steps + [(i.name,
i.type)], root, False); n += 1
                if n >= 4000: self.hier.insert(iid, 'end', text='…
more instances not listed (use Find)', values=('',)); break
    def hier_en_levels(self, start, budget):
        """expand the tree breadth-first, level by level, until
`budget` nodes exist (everything below stays lazy)"""
        tv = self.hier; level = list(start); made = 0
        while level and made < budget:
            nxt = []
            for iid in level:
                if tv.exists(iid + '#lazy'): self.hier_fill(iid)
                tv.item(iid, open=True)
                for ch in tv.get_children(iid):
                    if ch in self.hier_map: nxt.append(ch)
                made += len(tv.get_children(iid))
                if made >= budget: break
            level = nxt
        # whatever is open but still lazy gets its children now (they
are loaded when the user opens them)
    def on_hier_open(self, e):
        tv = self.hier; iid = tv.focus()
        if iid in self.hier_map: self.hier_fill(iid)
        for x in list(self.hier_map):               # the indicator
click may not have moved the focus
            if tv.exists(x + '#lazy') and tv.item(x, 'open'): self.hier_fill(x)
    def hier_expand_all(self):
        tv = self.hier;
self.hier_open_levels(list(tv.get_children('')), 10 ** 7)
    def hier_collapse_all(self):
        for iid in self.hier.get_children(''): self.hier.item(iid, open=False)
        for iid in list(self.hier_map):
            if self.hier.parent(iid) != '': self.hier.item(iid, open=False)
    def sync_hier(self):
        if not self.hier_map: return
        want = [s[0] for s in self.path]; root_iid = None
        for iid, (mn, steps, root) in self.hier_map.items():
            if not steps and mn == (self.root_mod or self.top) and
self.hier.parent(iid) == '': root_iid = iid; break
        if root_iid is None: return
        cur = root_iid
        for nm in want:
            self.hier_fill(cur); nxt = None
            for ch in self.hier.get_children(cur):
                ent = self.hier_map.get(ch)
                if ent and ent[1] and ent[1][-1][0] == nm: nxt = ch; break
            if nxt is None: return
            self.hier.item(cur, open=True); cur = nxt
        self.hier_lock = True; self.hier.selection_set(cur);
self.hier.see(cur); self.root.after(80, lambda: setattr(self,
'hier_lock', False))

    # ================= highlight / selection sets =================
    HL_COLORS = ['#e6194b', '#f58231', '#3cb44b', '#911eb4',
'#0082c8', '#f032e6', '#808000', '#008080']
    def resolve_group(self, color, objs, cur):
        nodes = set(); pins = set(); sigs = set()
        for f in objs:
            if f.path == cur:
                if f.kind == 'inst': nodes.add('i:' + f.name)
                elif f.kind == 'pin': pins.add(('i:' + f.name,
f.sub)); nodes.add('i:' + f.name)
                elif f.kind == 'net': sigs.add(f.name)
                elif f.kind in ('port', 'topport'): nodes.add('p:' + f.name)
            if f.kind == 'port' and f.path[:-1] == cur and f.path:
pins.add(('i:' + f.path[-1], f.name))
        return {'color': color, 'nodes': nodes, 'pins': pins, 'sigs': sigs}
    def hl_state(self, v):
        cur = self.cur_path(); key = (self.mod, cur, self.hl_ver,
self.theme_name)
        if self._hl_cache and self._hl_cache[0] == key: return self._hl_cache[1]
        groups = [self.resolve_group(c, objs, cur) for c, objs in
self.hl_groups]
        ms = self.resolve_group(self.T['acc'], self.msel, cur) if
self.msel else None
        ovr = {}
        for g in groups + ([ms] if ms else []):
            for s in g['sigs']: ovr[s] = g['color']
        res = (groups, ms, ovr); self._hl_cache = (key, res); return res
    def set_msel(self, objs):
        self.msel = list(objs) if objs else None; self.hl_ver += 1;
self.found_list = list(objs or []); self.found_idx = -1
        if objs: self.sel_node = self.sel_sig = self.sel_pin = None;
self.sel_cell = None
        self.refresh_inspector(); self.request_redraw(); self.sync_layout_sel()
        if objs: self.set_status('%d object%s selected. Zoom to
selection: Z   next match: F3' % (len(objs), '' if len(objs) == 1 else
's'))
    def add_highlight(self, objs):
        if not objs: return
        col = self.HL_COLORS[len(self.hl_groups) % len(self.HL_COLORS)]
        self.hl_groups.append((col, list(objs))); self.hl_ver += 1;
self.request_redraw(); self.set_status('Highlighted %d objects (%d
highlight sets). Edit > Clear highlights removes them.' % (len(objs),
len(self.hl_groups)))
    def clear_highlights(self):
        self.hl_groups = []; self.msel = None; self.hl_ver += 1;
self.refresh_inspector(); self.request_redraw();
self.set_status('Highlights and selection cleared.')
    def obj_bbox(self, v, f):
        def nb(key):
            nd = v.by_key.get(key)
            return (nd.x, nd.y, nd.x + nd.sym.w, nd.y + nd.sym.h) if
nd else None
        if f.kind == 'inst': return nb('i:' + f.name)
        if f.kind == 'pin':
            nd = v.by_key.get('i:' + f.name); q = pin_pos(nd, f.sub)
if nd else None
            return (q[0] - 20, q[1] - 14, q[0] + 20, q[1] + 14) if q
else nb('i:' + f.name)
        if f.kind in ('port', 'topport'): return nb('p:' + f.name)
        if f.kind == 'net':
            bb = None
            for s in ([v.sigs[f.name]] if f.name in v.sigs else []) +
[s for s in v.sig_list if s.base == f.name and s.id != f.name][:200]:
                ensure_geom(v, s)
                if s.polys:
                    b = s.bb; bb = b if bb is None else (min(bb[0],
b[0]), min(bb[1], b[1]), max(bb[2], b[2]), max(bb[3], b[3]))
            return bb
        return None
    def steps_for(self, root, path):
        mods = chain_modules(self.D, root, path); return [(path[i],
mods[i + 1].name) for i in range(len(path))], mods[-1].name
    def zoom_to(self, objs):
        if not objs or not self.D: return
        root = self.search_root or self.root_mod; cur =
self.cur_path(); sel = [f for f in objs if f.path == cur]
        if not sel:
            f = objs[0]
            try: steps, mn = self.steps_for(root, f.path)
            except EcoError as ex: messagebox.showwarning(APP_NAME,
str(ex)); return
            self.open_module(mn, steps, root); self.need_fit = False;
cur = self.cur_path(); sel = [x for x in objs if x.path == cur]
        if sel: self.ensure_page_for(sel[0])
        v = self.get_view(); bb = None
        for f in sel[:3000]:
            b = self.obj_bbox(v, f)
            if b: bb = b if bb is None else (min(bb[0], b[0]),
min(bb[1], b[1]), max(bb[2], b[2]), max(bb[3], b[3]))
        if bb is None: self.set_status('None of the selected objects
can be shown in this module.'); return
        W = max(self.canvas.winfo_width(), 100); H =
max(self.canvas.winfo_height(), 100); pad = 60
        w = bb[2] - bb[0] + 2 * pad; h = bb[3] - bb[1] + 2 * pad
        self.z = max(0.003, min(W / w, H / h, 2.0)); self.vx = bb[0] -
pad - (W / self.z - w) / 2; self.vy = bb[1] - pad - (H / self.z - h) /
2
        self.request_redraw()
    def zoom_to_selection(self):
        if self.msel: self.zoom_to(self.msel)
        elif self.hl_groups: self.zoom_to([f for c, o in
self.hl_groups for f in o])
        elif self.sel_node and self.get_view():
            nd = self.get_view().by_key.get(self.sel_node)
            if nd: self.center_on(nd.x + nd.sym.w / 2, nd.y + nd.sym.h / 2)
    def next_match(self, step=1):
        objs = self.found_list
        if not objs: self.set_status('No search results yet. Use Find
> Find objects.'); return
        self.found_idx = (self.found_idx + step) % len(objs); f =
objs[self.found_idx]
        self.msel = [f]; self.hl_ver += 1; self.zoom_to([f]);
self.refresh_inspector()
        self.set_status('Match %d of %d: %s' % (self.found_idx + 1,
len(objs), found_str(f)))
    def open_find(self, kind='inst'):
        if not self.D: messagebox.showinfo(APP_NAME, 'Load a netlist
first.'); return
        if self.find_win is not None and self.find_win.winfo_exists():
self.find_win.set_kind(kind); self.find_win.lift();
self.find_win.focus_force(); return
        self.find_win = FindDialog(self, kind)
    def find_dialog(self): self.open_find('inst')
    def insp_msel(self):
        objs = self.msel
        self.hdr('Selection'); ttk.Label(self.insp, text='%d object%s'
% (len(objs), '' if len(objs) == 1 else 's'), font=(self.mono,
11)).pack(anchor='w')
        lb = tk.Listbox(self.insp, height=14, activestyle='none',
relief='flat', bg=self.T['fill'], fg=self.T['ink'],
selectbackground=self.T['acc_soft'], selectforeground=self.T['ink'],
highlightthickness=1, highlightbackground=self.T['grid_major'],
font=(self.mono, 10))
        for f in objs[:2000]: lb.insert('end', found_str(f))
        lb.pack(fill='both', expand=True, pady=6)
        lb.bind('<Double-Button-1>', lambda e:
(self.zoom_to([objs[lb.curselection()[0]]]) if lb.curselection() else
None))
        f = ttk.Frame(self.insp); f.pack(fill='x')
        ttk.Button(f, text='Zoom to', command=lambda:
self.zoom_to(objs)).pack(side='left', padx=2); ttk.Button(f,
text='Highlight', command=lambda:
self.add_highlight(objs)).pack(side='left', padx=2)
        ttk.Button(f, text='Clear',
command=self.clear_highlights).pack(side='left', padx=2)
        ttk.Label(self.insp, text='Double-click a line to zoom to
it.', style='Hint.TLabel').pack(anchor='w', pady=4)
    def refresh_inspector(self):
        for w in self.insp.winfo_children(): w.destroy()
        if not self.D:
            ttk.Label(self.insp, text='Select an instance, pin or wire
to inspect it.\\nLoad a netlist to begin.',
style='Hint.TLabel').pack(anchor='w'); return
        v = self.get_view()
        if self.sel_cell and not self.sel_node and not self.sel_sig
and self.lib_symbol(self.sel_cell) is not None:
self.insp_cell(self.sel_cell); return
        if self.sel_node and v and self.sel_node in v.by_key:
            nd = v.by_key[self.sel_node]
            {'inst': self.insp_inst, 'port': self.insp_port, 'assign':
self.insp_assign}[nd.kind](nd); return
        if self.sel_sig and v and self.sel_sig in v.sigs:
self.insp_net(v.sigs[self.sel_sig]); return
        if self.msel: self.insp_msel(); return
        self.insp_module(self.cur_mod())

    # ================= Verilator log mapping =================
    def map_vl(self, items, lmap):
        for i in items:
            fn = os.path.basename(i.net or ''); i.net = ''
            hit = lmap.lookup(i.line) if (fn == 'design.v' and lmap is
not None) else None
            if hit: i.mod = hit[0]; i.inst = hit[1] or ''
        return items


HELP_TEXT = """ADDING INSTANCES
  Library tab: drag a cell or module onto the sheet, or double-click
it to drop it at the centre of the view.
  Right-click an empty spot on the sheet > Add instance here.

CONNECTING PINS (ECO)
  Drag from one pin to another, or use the ECO wire tool (W), or
Shift+click two pins.
  No net name is needed. A new net is created automatically and named
      <hierarchical path of the source pin>_eco_net_<n>      e.g.
u_core_u_alu_U1_ZN_eco_net_3
  (Tools > ECO net name style switches to the escaped form
\\/u_core/u_alu/U1/ZN_eco_net_3 ).

ACROSS HIERARCHY
  Click the first pin, then navigate to another module (hierarchy
tree, double-click a block, breadcrumbs)
  and click the second pin. The pending pin stays active and is shown
in the status bar.
  Netlist Studio punches the ports through every module boundary on the way:
      ECO_<module>_hier_port_<n>   (output on the source side, input
on the destination side)
  and creates the connecting net in the lowest common module. If a
module is shared by several instances
  you can uniquify it (private copy named <module>_eco_<k>) or add the
port to all instances.
  Everything is listed in the ECO log tab and can be undone with Ctrl+Z.

FINDING THINGS
  Find > Find objects by regexp (Ctrl+F). Choose instances, pins,
nets, module ports or top-level I/O ports,
  type a regular expression, and Select, Highlight or Zoom to the
results. The regexp is matched against the
  full hierarchical name (/u_core/u_alu/U12/ZN) unless "Match local
name only" is ticked.
  F3 / Shift+F3 step through the matches, Z zooms to the selection.

REUSING WIRES (no new net)
  Drag a pin onto a wire, or in ECO wire mode click a wire and then a
pin (in either order, even across modules).
  The existing wire is reused. A new <path>_eco_net_<n> is only
created when two pins deep in different
  modules need a connecting wire, or when neither end has a wire yet.

SAVING
  Save ECO netlist (Ctrl+S) keeps the module hierarchy. Modules you
did not touch are written back
  exactly as loaded; edited modules are regenerated. An .eco.log file
is written beside the netlist.

CHECKS
  Run checks: built-in structural and logic checks. Verilator lint:
runs the real Verilator if it is
  installed and shows its messages next to the built-in ones.
Double-click a problem to jump to it.

KEYS   Ctrl+Z / Ctrl+Shift+Z undo / redo    F fit    W ECO wire    Del
delete    Esc cancel
       Ctrl+F find    Ctrl+S save    mouse wheel zoom    drag the
background or middle-drag to pan
"""

def run_selftest():
    lib = parse_liberty(sample_lib_text()); src =
sample_netlist_text(); D = parse_verilog(src)
    assert write_design(D)[0] == src, 'round trip'
    log, created = eco_connect(D, lib, 'soc_top', HPin(('u_add',),
'u_fa3', 'S'), HPin(('u_flags',), 'u_nor0', 'A1'))
    txt = write_design(D)[0]
    assert 'ECO_adder4_hier_port_1' in txt and
'ECO_flag_logic_hier_port_2' in txt and 'u_add_u_fa3_S_eco_net_1' in
txt
    assert not D.modules['reg4'].dirty
    print('selftest OK (%d modules, %d ECO log lines)' %
(len(D.order), len(D.eco_log)))

class quiet_stderr:
    """silence file descriptor 2 (also for C libraries such as
fontconfig) inside the with-block"""
    def __enter__(self):
        self.fd = None
        if os.environ.get('NS_VERBOSE'): return self
        try:
            self.fd = os.dup(2); dn = os.open(os.devnull,
os.O_WRONLY); os.dup2(dn, 2); os.close(dn)
        except OSError: self.fd = None
        return self
    def __exit__(self, *a):
        if self.fd is not None:
            try: os.dup2(self.fd, 2); os.close(self.fd)
            except OSError: pass

def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv); args = {}
    if '--selftest' in argv: run_selftest(); return 0
    if '-h' in argv or '--help' in argv or '-help' in argv:
        print(__doc__); return 0
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ('--sample', '-sample'): args['sample'] = True
        elif a in ('--threads', '-threads', '-t'):
            try: THREADS[0] = max(0, int(argv[i + 1])); i += 1
            except (IndexError, ValueError): print('netlist-studio:
-threads needs a number (0 = all cores)'); return 2
        elif a in ('--lib', '--netlist', '-lib', '-netlist'):
            k = a.lstrip('-'); args.setdefault(k, [])
            while i + 1 < len(argv) and not argv[i +
1].startswith('-'): args[k].append(argv[i + 1]); i += 1
        i += 1
    try:
        import faulthandler; faulthandler.enable(open(CRASH_LOG, 'a'))
    except Exception: pass
    sys.setrecursionlimit(8000)
    try: threading.stack_size(64 * 1024 * 1024)
    except (ValueError, RuntimeError): pass
    qs = quiet_stderr().__enter__()    # host fontconfig files the
bundled library cannot parse print harmless warnings whenever a font
is first used; errors still go to the crash log and dialogs
(NS_VERBOSE=1 shows stderr)
    root = tk.Tk(); app = App(root, args); root.update_idletasks()
    def on_error(exc, val, tb):
        text = ''.join(traceback.format_exception(exc, val, tb));
log_crash(text)
        if isinstance(val, MemoryError): msg = 'Out of memory (peak %d
MB).' % mem_mb()
        else: msg = '%s: %s' % (exc.__name__, val)
        try: messagebox.showerror(APP_NAME, 'An internal error
occurred. Your session is still open.\n\n%s\n\nDetails were written to
%s' % (msg, CRASH_LOG))
        except Exception: pass
    root.report_callback_exception = on_error
    try: root.mainloop()
    finally: qs.__exit__()
    return 0

if __name__ == '__main__':
    sys.exit(main())
