#!/usr/bin/env python3
# Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Generate the ROS-free parameter struct of the planning core from the table.

    generate_core_params.py TABLE.yaml OUT.hpp

TABLE.yaml is the generate_parameter_library table. OUT.hpp declares, in
namespace geodex_nav2_planner,

  struct Se2Params             every parameter with its default
  kParameterNames              every parameter name, in table order
  assign_params(to, from)      copy every field from a struct with the same names
  set_param(p, key, value)     parse one field from text, with the table's bounds
  params_equal(a, b)           field-wise equality
  params_to_string(p)          one "key: value" line per parameter

The core and the ROS plugin share this one list.
"""

import math
import re
import sys

import yaml

LICENSE_HEADER = [
    '// Copyright 2026 Space and Terrestrial Autonomous Robotic Systems (STARS) Lab',
    '//',
    '// Licensed under the Apache License, Version 2.0 (the "License");',
    '// you may not use this file except in compliance with the License.',
    '// You may obtain a copy of the License at',
    '//',
    '//     http://www.apache.org/licenses/LICENSE-2.0',
    '//',
    '// Unless required by applicable law or agreed to in writing, software',
    '// distributed under the License is distributed on an "AS IS" BASIS,',
    '// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.',
    '// See the License for the specific language governing permissions and',
    '// limitations under the License.',
]

CXX_TYPE = {'double': 'double', 'int': 'std::int64_t', 'bool': 'bool', 'string': 'std::string'}


def cxx_literal(kind, value):
    if kind == 'double':
        v = float(value)
        if not math.isfinite(v):
            raise SystemExit(f'non-finite default {value}')
        text = repr(v)
        return text if re.search(r'[.eE]', text) else text + '.0'
    if kind == 'int':
        return str(int(value))
    if kind == 'bool':
        return 'true' if value else 'false'
    if kind == 'string':
        return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"') + '"'
    raise SystemExit(f'unsupported type {kind}')


def doc_lines(text, indent):
    words = ' '.join(str(text).split()).split(' ')
    lines, line = [], ''
    for w in words:
        if len(line) + len(w) + 1 > 90:
            lines.append(line)
            line = w
        else:
            line = (line + ' ' + w).strip()
    if line:
        lines.append(line)
    return [f'{indent}/// {text}' for text in lines]


def main(table_path, out_path):
    with open(table_path) as f:
        doc = yaml.safe_load(f)
    (namespace, entries), = doc.items()
    params = []
    for name, spec in entries.items():
        kind = spec['type']
        if kind not in CXX_TYPE:
            raise SystemExit(f'{name}: type {kind} does not have a core equivalent')
        bounds = (spec.get('validation') or {}).get('bounds<>')
        params.append((name, kind, spec['default_value'], spec.get('description', ''), bounds))

    o = []
    o.extend(LICENSE_HEADER)
    o.append('')
    o.append('// Generated from %s by generate_core_params.py. Do not edit.'
             % table_path.split('/')[-1])
    o.append('#pragma once')
    o.append('')
    o.append('#include <array>')
    o.append('#include <cerrno>')
    o.append('#include <cstdint>')
    o.append('#include <cstdlib>')
    o.append('#include <string>')
    o.append('#include <string_view>')
    o.append('')
    o.append(f'namespace {namespace} {{')
    o.append('')
    o.append("/// @brief Every parameter of the planner, with the table's defaults.")
    o.append('struct Se2Params {')
    for name, kind, default, desc, _ in params:
        o.extend(doc_lines(desc, '  '))
        o.append(f'  {CXX_TYPE[kind]} {name} = {cxx_literal(kind, default)};')
    o.append('};')
    o.append('')
    o.append('/// @brief Every parameter name, in table order.')
    o.append(f'inline constexpr std::array<std::string_view, {len(params)}> kParameterNames = {{')
    for name, *_ in params:
        o.append(f'    "{name}",')
    o.append('};')
    o.append('')
    o.append('/// @brief Copies every parameter from a struct with the same field names.')
    o.append('template <typename From>')
    o.append('void assign_params(Se2Params& to, const From& from) {')
    for name, kind, *_ in params:
        if kind == 'string':
            o.append(f'  to.{name} = std::string(from.{name});')
        else:
            o.append(f'  to.{name} = from.{name};')
    o.append('}')
    o.append('')
    o.append('/// @brief True when every parameter of `a` equals that of `b`.')
    o.append('inline bool params_equal(const Se2Params& a, const Se2Params& b) {')
    terms = [f'a.{name} == b.{name}' for name, *_ in params]
    o.append('  return ' + '\n      && '.join(terms) + ';')
    o.append('}')
    o.append('')
    o.append("/// @brief Parses one parameter from text and checks it against the table's")
    o.append('/// bounds. Returns false for an unknown name, unparsable text or a value out')
    o.append('/// of bounds, and leaves the struct unchanged then.')
    o.append('inline bool set_param(Se2Params& p, std::string_view key, '
             'std::string_view value) {')
    o.append('  const std::string text(value);')
    o.append('  const auto as_double = [&](double& out) {')
    o.append('    char* end = nullptr;')
    o.append('    errno = 0;')
    o.append('    out = std::strtod(text.c_str(), &end);')
    o.append('    return !text.empty() && errno == 0 && end == text.c_str() + text.size();')
    o.append('  };')
    o.append('  const auto as_int = [&](std::int64_t& out) {')
    o.append('    char* end = nullptr;')
    o.append('    errno = 0;')
    o.append('    out = std::strtoll(text.c_str(), &end, 10);')
    o.append('    return !text.empty() && errno == 0 && end == text.c_str() + text.size();')
    o.append('  };')
    o.append('  const auto as_bool = [&](bool& out) {')
    o.append('    if (text == "true" || text == "True" || text == "1") return out = true, true;')
    o.append('    if (text == "false" || text == "False" || text == "0") '
             'return out = false, true;')
    o.append('    return false;')
    o.append('  };')
    for name, kind, _, _, bounds in params:
        o.append(f'  if (key == "{name}") {{')
        if kind == 'string':
            o.append(f'    p.{name} = text;')
            o.append('    return true;')
        else:
            var = {'double': 'double v = 0.0;', 'int': 'std::int64_t v = 0;',
                   'bool': 'bool v = false;'}[kind]
            fn = {'double': 'as_double', 'int': 'as_int', 'bool': 'as_bool'}[kind]
            o.append(f'    {var}')
            cond = f'!{fn}(v)'
            if bounds is not None:
                lo, hi = (cxx_literal(kind, b) for b in bounds)
                cond += f' || v < {lo} || v > {hi}'
            o.append(f'    if ({cond}) return false;')
            o.append(f'    p.{name} = v;')
            o.append('    return true;')
        o.append('  }')
    o.append('  return false;')
    o.append('}')
    o.append('')
    o.append('/// @brief One "key: value" line per parameter, for logs.')
    o.append('inline std::string params_to_string(const Se2Params& p) {')
    o.append('  std::string s;')
    o.append('  const auto num = [](double v) {')
    o.append('    std::string t = std::to_string(v);')
    o.append("    while (t.size() > 1 && t.back() == '0' && t[t.size() - 2] != '.') t.pop_back();")
    o.append('    return t;')
    o.append('  };')
    for name, kind, *_ in params:
        if kind == 'double':
            val = f'num(p.{name})'
        elif kind == 'int':
            val = f'std::to_string(p.{name})'
        elif kind == 'bool':
            val = f'(p.{name} ? "true" : "false")'
        else:
            val = f'p.{name}'
        o.append(f'  s += "{name}: ";')
        o.append(f'  s += {val};')
        o.append('  s += "\\n";')
    o.append('  return s;')
    o.append('}')
    o.append('')
    o.append(f'}}  // namespace {namespace}')
    o.append('')

    with open(out_path, 'w') as f:
        f.write('\n'.join(o))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    main(sys.argv[1], sys.argv[2])
