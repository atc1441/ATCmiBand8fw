'''
rsonlite -- an extremely lightweight version of rson.

Copyright (c) 2012, Patrick Maupin

License :: MIT

http://pypi.python.org/pypi/rsonlite
http://code.google.com/p/rson/

rsonlite makes it easy to build a file parser for
declarative hierarchical data structures using indentation.
(Spaces only, tabs not considered indentation.)

The only special characters are '#', '=', and indentation:

  - Indentation denotes a key/value relationship.  The
    value is indented from the key.

  - = Denotes the start of a free-format string.  These
      strings can contain '=' and '#' characters, and
      even be multi-line, but every line in the string
      must be indented past the initial equal sign.

      Note that, for multi-line strings, indentation is
      preserved but normalized such that at least one
      line starts in the left column.  This allows for
      restructuredText or Python code to exist inside
      multi-line strings.

  - # Denotes the start of a line comment, when not
      inside a free-format string.

The only Python objects resulting from parsing a file
with rsonlite are:

  - strings:
        free-format strings (described above) can
        contain any character, but the whitespace
        before/after the string may be stripped.

        Regular strings must fit on a single line and
        cannot contain '=' or '#' characters.

        Regular strings may be used as keys in key/value
        pairs, but free-format strings may not.

   - tuple:
        A key/value pair is a two-element tuple.  The key is always
        a string.  The value is always a list.

   - list:
        The top level is a list, and the value element of every
        key/value pair tuple is also a list.  Lists can contain
        strings and key/value pair tuples.
'''

import re

version = __version__ = '0.1.0'

# Our attempt at rationalizing differences between Python 2 and Python 3.

try:
    basestring
except NameError:
    basestring = str
    class unicode: pass

# Use OrderedDict if it's available

try:
    from collections import OrderedDict as stddict
except ImportError:
    stddict = dict

# Splits the entire file into probable tokens.

splitter = re.compile('(\n *|=[^\n]*|#[^\n]*|[^\n#=]+)').findall

class RsonToken(str):
    ''' A string that may be annotated with location information
    '''
    def __new__(cls, s, line, col):
        self = str.__new__(cls, s)
        self.line = line
        self.col = col
        return self
    def __add__(self, other):
        return RsonToken(str(self) + other, self.line, self.col)


def gettoks(source):
    ''' Convert string into (probable) tokens
         (some tokens may be recombined later, e.g. if they
          contain # or = but were already inside a string)
    '''

    # Use "regular" strings, whatever that means for the given Python
    if isinstance(source, unicode):
        source = source.encode('utf-8', 'replace')
    elif not isinstance(source, basestring):
        source = source.decode('utf-8', 'replace')

    # Convert MS-DOS or Mac line endings to the one true way, and
    # prefix the source with a linefeed to simplify the tokenization.
    source = '\n' + source.replace('\r\n', '\n').replace('\r', '\n')

    line = 0
    for tok in splitter(source):
        if tok.startswith('\n'):
            line += 1
            col = len(tok)
        else:
            yield RsonToken(tok, line, col)
            col += len(tok)

def multiline(lineinfo, dedent):
    ''' Returns one string for each line,
        properly dedented.
    '''
    linenum = lineinfo[0].line
    for tok in lineinfo:
        while linenum < tok.line:
            yield ''
            linenum += 1
        yield (tok.col - dedent) * ' ' + tok.rstrip()
        linenum += 1

def getfreeformat(toklist, firsttok, firstcol):
    ''' Returns a free-formatted string.
    '''
    curline = firsttok.line
    firstpart = firsttok[1:].strip()  # Get past = sign
    lineinfo = []
    while toklist and toklist[-1].col > firstcol:
        tok = toklist.pop()
        if tok.line == curline:
            lineinfo[-1] += tok
        else:
            lineinfo.append(tok)
            curline = tok.line
    if lineinfo:
        dedent = min(tok.col for tok in lineinfo)
        if firstpart:
            lineinfo.insert(0, RsonToken(firstpart, firsttok.line, dedent))
        firstpart = '\n'.join(multiline(lineinfo, dedent))
    return RsonToken(firstpart, firsttok.line, firsttok.col)

def loads(source):
    '''  load a string into an rsonlite datastructure.
         If the source is not a string instance, then
         loads will attempt to convert it into a string
         instance, by encoding to UTF-8 on Python 2,
         or decoding from UTF-8 on Python 3.
    '''
    toklist = list(gettoks(source))
    toklist.reverse()
    result = [None]
    stack = []
    curcol = -1
    curlist = result
    while toklist:
        tok = toklist.pop()
        if tok.startswith('#'):
            continue
        col = tok.col
        if col > curcol:
            stack.append((curcol, curlist))
            oldlist = curlist
            curcol, curlist = col, []
            oldlist[-1] = oldlist[-1], curlist
        while col < curcol:
            curcol, curlist = stack.pop()
        if col != curcol:
            err = IndentationError('unindent does not match any outer indentation level')
            err.filename = '<rsonlite>'
            err.lineno = tok.line
            raise err
        if tok.startswith('='):
            curlist.append(getfreeformat(toklist, tok, col))
        else:
            curlist.append(RsonToken(tok.rstrip(), tok.line, tok.col))
            if toklist and toklist[-1].line == tok.line:
                tok = toklist.pop()
                if tok.startswith('='):
                    curlist[-1] = curlist[-1], [getfreeformat(toklist, tok, col)]
                else:
                    assert tok.startswith('#')  # else problem in regex...
    result, = result
    return [] if result is None else result[1]

def dumps(data, indent='    ', initial_indent=''):
    ''' Dump a string loaded with loads back out.
    '''
    def getstring(data, indent2):
        if '\n' in data:
            data = ('\n'+indent2).join([''] + data.split('\n'))
        return data

    def recurse(data, indent2):
        assert isinstance(data, list), repr(data)
        for data in data:
            if isinstance(data, tuple):
                key, value = data
                if len(value) == 1 and isinstance(value[0], basestring):
                    append('%s%s = %s' % (indent2, key, getstring(value[0], indent2+indent)))
                else:
                    append('%s%s' % (indent2, key))
                    recurse(value, indent2 + indent)
            else:
                assert isinstance(data, basestring)
                if '\n' in data or '=' in data or '#' in data:
                    append(indent2 + '=')
                    append(getstring(data, indent2 + '    '))
                else:
                    append('%s%s' % (indent2, data))
    result = []
    append = result.append
    recurse(data, initial_indent)
    append('')
    return '\n'.join(result)

def pretty(data, indent='    '):
    ''' Pretty-print a string loaded by loads into
        something that makes it easy to see the actual
        structure of the data.  The return value of
        this should be parseable by eval()
    '''
    def recurse(data, indent2):
        assert isinstance(data, list)
        for data in data:
            assert isinstance(data, (tuple, basestring))
            if isinstance(data, tuple) and (
                       len(data[1]) != 1 or not isinstance(data[1][0], basestring)):
                append('%s(%s, [' % (indent2, repr(data[0])))
                recurse(data[1], indent2 + indent)
                append('%s])' % (indent2))
            else:
                append('%s%s,' % (indent2, repr(data)))
    result = []
    append = result.append
    append('[')
    recurse(data, indent)
    append(']')
    append('')
    return '\n'.join(result)

##########################################################################
# These higher-level functions might suffice for simple data, and also
# provide a template for designing similar functions.

def stringparse(s, special=dict(true=True, false=False, null=None)):
    ''' This gives an example of handling the JSON special identifiers
        true, false and null, and also of handling simple arrays.
    '''
    if s in special:
        return special[s]
    if s.startswith('[') and s.endswith(']'):
        t = s[1:-1]
        for ch in '"\'[]{}\n':
            if ch in t:
                return s
        return [x.strip() for x in t.split(',')]
    return s

def simpleparse(source, stringparse=stringparse, stddict=stddict):
    ''' Return the simplest structure that uses dicts instead
        of tuples, and doesn't lose any source information.
        Use ordered dicts if they are available.
    '''
    def recurse(mylist):
        if len(mylist) == 1 and isinstance(mylist[0], basestring):
            return stringparse(mylist[0])
        keys = [x[0] for x in mylist if isinstance(x, tuple)]
        if not keys:
            return mylist  # simple list
        if len(set(keys)) == len(mylist):
            return stddict((x, recurse(y)) for (x, y) in mylist)
        # Complicated.  Make a list that might have multiple dicts
        result = []
        curdict = None
        for item in mylist:
            if not isinstance(item, tuple):
                result.append(stringparse(item))
                curdict = None
                continue
            key, value = item
            if curdict is None or key in curdict:
                curdict = stddict()
                result.append(curdict)
            curdict[key] = recurse(value)
        return result
    return recurse(source if isinstance(source, list) else loads(source))
