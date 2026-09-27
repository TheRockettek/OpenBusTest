// overrides.h
//
// Built-in macro overrides: map of OSC macro name -> Lua body to inline.
//
// The Lua body is inserted verbatim as the function body (indented 2 spaces).
// It uses the host-provided stack helpers.
//
// Convention matching the OSC originals:
//   Matrix_CharToLowerCase / Matrix_CharToUpperCase
//     Input:  top of _ss = single character string
//     Output: top of _ss = converted character
//             _pushf(<1 if changed, 0 if unchanged>)
//
//   Matrix_CharToLowerCaseCE
//     Same contract, also handles CE (Windows-1252) special chars.

#pragma once
#include <string>
#include <map>

inline std::map<std::string, std::string> get_builtin_overrides() {
    std::map<std::string, std::string> m;

    // ------------------------------------------------------------------
    // Matrix_CharToLowerCase
    // Converts the top string-stack character to lowercase (A-Z + umlauts).
    // Pushes 1 onto _fs if the character was changed, else 0.
    // ------------------------------------------------------------------
    m["Matrix_CharToLowerCase"] = R"LUA(
  local _char_lower = {
    A="a", B="b", C="c", D="d", E="e", F="f", G="g", H="h",
    I="i", J="j", K="k", L="l", M="m", N="n", O="o", P="p",
    Q="q", R="r", S="s", T="t", U="u", V="v", W="w", X="x",
    Y="y", Z="z",
    ["\xC4"]="\xE4",  -- Ä -> ä
    ["\xD6"]="\xF6",  -- Ö -> ö
    ["\xDC"]="\xFC",  -- Ü -> ü
  }
  do
    local s = _pops()
    local r = _char_lower[s]
    if r then
      _pushs(r)
      _pushf(1)
    else
      _pushs(s)
      _pushf(0)
    end
  end
)LUA";

    // ------------------------------------------------------------------
    // Matrix_CharToUpperCase
    // Converts the top string-stack character to uppercase (a-z + umlauts).
    // Pushes 1 onto _fs if the character was changed, else 0.
    // ------------------------------------------------------------------
    m["Matrix_CharToUpperCase"] = R"LUA(
  local _char_upper = {
    a="A", b="B", c="C", d="D", e="E", f="F", g="G", h="H",
    i="I", j="J", k="K", l="L", m="M", n="N", o="O", p="P",
    q="Q", r="R", s="S", t="T", u="U", v="V", w="W", x="X",
    y="Y", z="Z",
    ["\xE4"]="\xC4",  -- ä -> Ä
    ["\xF6"]="\xD6",  -- ö -> Ö
    ["\xFC"]="\xDC",  -- ü -> Ü
  }
  do
    local s = _pops()
    local r = _char_upper[s]
    if r then
      _pushs(r)
      _pushf(1)
    else
      _pushs(s)
      _pushf(0)
    end
  end
)LUA";

    // ------------------------------------------------------------------
    // Matrix_CharToLowerCaseCE
    // Like Matrix_CharToLowerCase but also handles additional CE
    // (Central European, Windows-1252) accented characters.
    // Calls the base lowercase first, then applies CE-specific mappings.
    // ------------------------------------------------------------------
    m["Matrix_CharToLowerCaseCE"] = R"LUA(
  -- CE extra mappings (Windows-1252 code points)
  local _char_lower_ce = {
    -- Accented Latin capitals (CP-1252 / ISO-8859-2 range)
    ["\xC0"]="\xE0", ["\xC1"]="\xE1", ["\xC2"]="\xE2", ["\xC3"]="\xE3",
    ["\xC5"]="\xE5", ["\xC6"]="\xE6", ["\xC7"]="\xE7", ["\xC8"]="\xE8",
    ["\xC9"]="\xE9", ["\xCA"]="\xEA", ["\xCB"]="\xEB", ["\xCC"]="\xEC",
    ["\xCD"]="\xED", ["\xCE"]="\xEE", ["\xCF"]="\xEF", ["\xD0"]="\xF0",
    ["\xD1"]="\xF1", ["\xD2"]="\xF2", ["\xD3"]="\xF3", ["\xD4"]="\xF4",
    ["\xD5"]="\xF5", ["\xD8"]="\xF8", ["\xD9"]="\xF9", ["\xDA"]="\xFA",
    ["\xDB"]="\xFB", ["\xDD"]="\xFD", ["\xDE"]="\xFE",
  }
  -- First apply the base ASCII + umlaut conversion
  macro_Matrix_CharToLowerCase()
  -- Then apply CE-specific mapping if base didn't match (result flag is top of _fs)
  do
    local changed = _popf()
    if changed == 0 then
      local s = _pops()
      local r = _char_lower_ce[s]
      if r then
        _pushs(r)
        changed = 1
      else
        _pushs(s)
      end
    end
    _pushf(changed)
  end
)LUA";

    return m;
}
