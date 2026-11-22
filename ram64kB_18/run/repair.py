#!/usr/bin/env python3
"""Rebuild 1.py from the hard-wrapped 1.py.orig.

The source was wrapped greedily at column 70: the rest of each long line was
put on the next line with no indentation.  Joining those pieces back with a
single space restores the original logical lines (the first piece kept the
real indentation of the statement).
"""
import re
import sys


def split_comment(s):
    """Return (code, comment) splitting at the first # outside a string."""
    i = 0
    n = len(s)
    q = None
    while i < n:
        c = s[i]
        if q:
            if c == '\\':
                i += 2
                continue
            if c == q:
                q = None
            i += 1
            continue
        if c in ('"', "'"):
            q = c
            i += 1
            continue
        if c == '#':
            return s[:i], s[i:]
        i += 1
    return s, None


def open_string(s):
    """True when s leaves a single-quoted string open at the end."""
    i = 0
    n = len(s)
    q = None
    while i < n:
        c = s[i]
        if q:
            if c == '\\':
                i += 2
                continue
            if c == q:
                q = None
            i += 1
            continue
        if c == '#':
            return False
        if c in ('"', "'"):
            # a triple quote may legitimately span lines; ignore it
            if s[i:i + 3] == c * 3:
                j = s.find(c * 3, i + 3)
                if j < 0:
                    return False
                i = j + 3
                continue
            q = c
        i += 1
    return q is not None


def wrap_join(lines):
    out = []
    i = 0
    dedent_start = re.compile(
        r'^(def |class |@|import |from |if __name__|#)')
    while i < len(lines):
        cur = lines[i]
        first_indent = len(cur) - len(cur.lstrip(' \t'))
        plen = len(cur)          # length of the last physical line joined
        while i + 1 < len(lines):
            nxt = lines[i + 1]
            if not nxt:
                break
            if open_string(cur):
                # a single-quoted string was cut in half: the next physical
                # line belongs to it whatever its indentation looks like
                cur = cur + ('' if nxt[:1] == ' ' else ' ') + nxt
                plen = len(nxt)
                i += 1
                continue
            if nxt[0] in ' \t':
                break
            m = re.match(r'\S+', nxt)
            if not m:
                break
            word = m.group(0)
            if cur.endswith('\\'):
                joined = cur[:-1] + nxt
            else:
                joined = cur + ' ' + nxt
            if not (plen <= 70 and plen + 1 + len(word) > 70):
                break
            if first_indent == 0 and not open_string(cur):
                # at top level a physical line can be a statement of its
                # own; only follow an obvious continuation
                punct = (',', '\\', '(', '[', '{', '+', '-', '*', '/', '%',
                         '&', '|', '^', '=', '<', '>', '.', ':', ';')
                if not (cur.endswith(punct) or
                        re.search(r'\b(and|or|not|if|else|in)$', cur)):
                    break
            if first_indent and dedent_start.match(nxt):
                break
            if cur.lstrip().startswith('#'):
                body = cur.lstrip()[1:]
                if (set(body) <= set('=-* ') or nxt.lstrip().startswith('#')):
                    break
                cur = joined
                plen = len(nxt)
                i += 1
                continue
            code, comment = split_comment(cur)
            if comment is not None:
                # a comment wrapped at the margin: its text continues
                cur = joined
                plen = len(nxt)
                i += 1
                continue
            if nxt.lstrip().startswith('#'):
                break
            cur = joined
            plen = len(nxt)
            i += 1
        out.append(cur)
        i += 1
    return out


FIXES = [
    # an earlier edit glued c.model onto c.memory and tab-indented the rest
    ("""        elif ch.type == 'memory': c.memory = True c.model = 'ECSM'
\t\t  if c.flags['ecsm'] else 'CCS' 
\t\t  if c.flags['ccs'] else 'NLDM' 
\t\t  if c.flags['nldm'] else 'none'
        return c""",
     """        elif ch.type == 'memory': c.memory = True
    c.model = ('ECSM' if c.flags['ecsm'] else 'CCS'
               if c.flags['ccs'] else 'NLDM'
               if c.flags['nldm'] else 'none')
    return c"""),
    # small text corruptions (random characters eaten by the damage)
    ("highlightthickness=0=app.T['bg']", "highlightthickness=0, bg=app.T['bg']"),
    ("W = m self.cv.winfo_width());", "W = max(50, self.cv.winfo_width());"),
    ("label='Redo'lerator='Ctrl+Shift+Z'",
     "label='Redo', accelerator='Ctrl+Shift+Z'"),
    ("self.hier_map           mn, steps, root",
     "self.hier_map: mn, steps, root"),
    ("label='Save Eetlist as…'", "label='Save netlist as…'"),
    ("accelerator='CtrF'", "accelerator='Ctrl+F'"),
    ("commanm_to_selection", "command=self.zoom_to_selection"),
    ("tl.add_command(lal='ECO console (Tcl / Python)'",
     "tl.add_command(label='ECO console (Tcl / Python)'"),
    ("missing fm the libraries", "missing from the libraries"),
]


PRE_FIXES = [
    # a lost opening quote used to keep the joiner running for hundreds of
    # lines (it thought the string was still open)
    ("\norne yet'))", "\nor 'none yet'))"),
]


def main():
    src = open('1.py.orig').read()
    for old, new in PRE_FIXES:
        if old not in src:
            print('PRE-FIX NOT FOUND:', repr(old[:60]))
        src = src.replace(old, new)
    lines = src.split('\n')
    lines = wrap_join(lines)
    src = '\n'.join(lines)
    for old, new in FIXES:
        if old not in src:
            print('FIX NOT FOUND:', repr(old[:60]))
        src = src.replace(old, new)
    open('1_fixed.py', 'w').write(src)
    print('wrote 1_fixed.py:', src.count('\n') + 1, 'lines')


if __name__ == '__main__':
    main()
