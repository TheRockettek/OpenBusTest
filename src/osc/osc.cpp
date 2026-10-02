// osc.cpp  -  OMSI .osc  ->  Lua converter
//
// Generated Lua uses two module-level stacks:
//   _fs  = float/number stack  (table, index 1..N, top at N)
//   _ss  = string stack
//
// Variable/constant access is delegated to host-provided functions:
//   get_local_var(name)      set_local_var(name, v)
//   get_sys_var(name)
//   get_local_str(name)      set_local_str(name, v)
//   get_const(name)
//   call_func(name, x)       -> y  (piecewise-linear curve lookup)
//   sound_trigger(name)
//   sound_trigger_file(name, filepath)
//   sys_macro_<name>()       (M.V.name system macros)
//   omsi_debug(s)            ($msg)

#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <stdexcept>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <windows.h>
#include <psapi.h>

#include "overrides.h"
#include "OscConverter.h"

// ============================================================
// TOKENIZER
// ============================================================

enum class TT {
    Number,
    String,
    BlockKW, // {frame}, {if}, {end}, {macro:name}, ...
    Command, // (L.L.var), (M.L.name), ...
    Ident,   // operators: +, -, sin, d, s0, &&, $+, ...
    Comment, // 'text
    Eof
};

struct Token {
    TT type;
    std::string val;
    int line = 0;
};

class Tokenizer {
    std::string src;
    size_t pos = 0;
    int line = 1;

    bool at_end() const {
        return pos >= src.size();
    }
    char cur() const {
        return src[pos];
    }

    void skip_ws() {
        while (!at_end() && std::isspace((unsigned char)cur())) {
            if (cur() == '\n')
                ++line;
            ++pos;
        }
    }

  public:
    explicit Tokenizer(std::string s) : src(std::move(s)) {}

    std::vector<Token> tokenize() {
        std::vector<Token> toks;
        size_t last_pos = (size_t)-1;

        while (true) {
            skip_ws();
            if (at_end())
                break;

            // infinite-loop guard
            if (pos == last_pos) {
                std::cerr << "Tokenizer stuck at pos=" << pos << " line=" << line << " char=0x"
                          << std::hex << (int)(unsigned char)src[pos] << std::dec << "\n";
                ++pos;
                continue;
            }
            last_pos = pos;

            int l = line;
            char c = cur();

            // ---- comment  '....
            if (c == '\'') {
                ++pos;
                std::string text;
                while (!at_end() && src[pos] != '\n')
                    text += src[pos++];
                toks.push_back({TT::Comment, text, l});
                continue;
            }

            // ---- block keyword  {xxx}
            if (c == '{') {
                ++pos;
                std::string kw;
                while (!at_end() && src[pos] != '}')
                    kw += src[pos++];
                if (!at_end())
                    ++pos;
                // normalise case for fixed keywords, but preserve macro/trigger names
                auto lower_prefix = [](std::string s, size_t prefix_len) {
                    for (size_t i = 0; i < prefix_len && i < s.size(); ++i)
                        s[i] = (char)std::tolower((unsigned char)s[i]);
                    return s;
                };
                // lowercase everything for keyword matching
                std::string lkw = kw;
                for (char& ch : lkw)
                    ch = (char)std::tolower((unsigned char)ch);
                // keep original case for macro:/trigger: suffix after the colon
                if (kw.size() >= 7 && lkw.substr(0, 6) == "macro:")
                    lkw = "macro:" + kw.substr(6);
                else if (kw.size() >= 9 && lkw.substr(0, 8) == "trigger:")
                    lkw = "trigger:" + kw.substr(8);
                toks.push_back({TT::BlockKW, lkw, l});
                continue;
            }

            // ---- command  (X.Y.name)  -- name may contain nested parens e.g. (S.L.Font_(8-1)x3)
            if (c == '(') {
                ++pos;
                std::string cmd;
                int depth = 1;
                while (!at_end() && depth > 0) {
                    if (src[pos] == '(') {
                        ++depth;
                        cmd += src[pos++];
                    } else if (src[pos] == ')') {
                        --depth;
                        if (depth > 0)
                            cmd += src[pos];
                        ++pos;
                    } else {
                        cmd += src[pos++];
                    }
                }
                toks.push_back({TT::Command, cmd, l});
                continue;
            }

            // ---- stray ')' - skip
            if (c == ')') {
                std::cerr << "Warning: stray ')' at line " << l << " - skipping\n";
                ++pos;
                continue;
            }

            // ---- string literal  "..."
            // NOTE: backslash is NOT an escape character in OSC (it's used for
            // Windows paths like "..\..\Folder\"), so we read until the next
            // bare '"' with no escape processing.
            if (c == '"') {
                ++pos;
                std::string s;
                while (!at_end() && src[pos] != '"') {
                    if (src[pos] == '\n')
                        ++line;
                    s += src[pos++];
                }
                if (!at_end())
                    ++pos;
                toks.push_back({TT::String, s, l});
                continue;
            }

            // ---- number:  optional leading '-' then digits
            //  '-' alone (without digit following) is the subtraction operator
            bool is_num =
                std::isdigit((unsigned char)c) ||
                (c == '.' && pos + 1 < src.size() && std::isdigit((unsigned char)src[pos + 1])) ||
                (c == '-' && pos + 1 < src.size() && std::isdigit((unsigned char)src[pos + 1]));
            if (is_num) {
                std::string num;
                if (c == '-')
                    num += src[pos++];
                while (!at_end() && (std::isdigit((unsigned char)src[pos]) || src[pos] == '.'))
                    num += src[pos++];
                // scientific notation
                if (!at_end() && (src[pos] == 'e' || src[pos] == 'E')) {
                    num += src[pos++];
                    if (!at_end() && (src[pos] == '+' || src[pos] == '-'))
                        num += src[pos++];
                    while (!at_end() && std::isdigit((unsigned char)src[pos]))
                        num += src[pos++];
                }
                toks.push_back({TT::Number, num, l});
                continue;
            }

            // ---- stray '}' (not part of a {kw} pair) - skip it
            if (c == '}') {
                std::cerr << "Warning: stray '}}' at line " << l << " - skipping\n";
                ++pos;
                continue;
            }

            // ---- identifier / operator  (everything else until whitespace/special)
            {
                std::string id;
                while (!at_end() && !std::isspace((unsigned char)src[pos]) && src[pos] != '{' &&
                       src[pos] != '}' && src[pos] != '(' && src[pos] != ')' && src[pos] != '\'' &&
                       src[pos] != '"') {
                    id += src[pos++];
                }
                if (!id.empty())
                    toks.push_back({TT::Ident, id, l});
            }
        }

        toks.push_back({TT::Eof, "", line});
        return toks;
    }
};

// ============================================================
// EMITTER
// ============================================================

static std::string lua_escape(const std::string& s) {
    std::string out;
    for (unsigned char c : s) {
        if (c == '"')
            out += "\\\"";
        else if (c == '\\')
            out += "\\\\"; // literal backslash -> escaped in Lua
        else if (c == '\n')
            out += "\\n";
        else if (c == '\r')
            out += "\\r";
        else
            out += (char)c;
    }
    return out;
}

static std::string ind(int depth) {
    return std::string(depth * 2, ' ');
}

static std::string sanitize(const std::string& s) {
    std::string out;
    for (unsigned char c : s) {
        if (std::isalnum(c))
            out += (char)std::tolower(c);
        else
            out += '_';
    }
    // collapse runs of underscores
    std::string r;
    bool last_under = false;
    for (char c : out) {
        if (c == '_') {
            if (!last_under)
                r += c;
            last_under = true;
        } else {
            r += c;
            last_under = false;
        }
    }
    while (!r.empty() && r.back() == '_')
        r.pop_back();
    if (r.empty())
        r = "x";
    return r;
}

static std::string lower_name(const std::string& name) {
    std::string result = name;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return result;
}

static std::string func_name(const std::string& kw) {
    if (kw == "frame")
        return "frame";
    if (kw == "frame_ai")
        return "frame_ai";
    if (kw == "init")
        return "init";
    if (kw.size() > 6 && kw.substr(0, 6) == "macro:")
        return "macro_" + sanitize(kw.substr(6));
    if (kw.size() > 8 && kw.substr(0, 8) == "trigger:")
        return "trigger_" + sanitize(kw.substr(8));
    return "block_" + sanitize(kw);
}

class Emitter {
    std::vector<Token> toks;
    size_t pos = 0;
    std::set<std::string> overrides; // runtime-dispatch overrides (--override flag)
    std::map<std::string, std::string> builtin_overrides; // inline-body overrides (overrides.h)

    // Expression stack: accumulates pending float expressions instead of
    // materialising them immediately.  Binary/unary ops fold the top 1-2
    // entries into a single combined Lua expression, eliminating most
    // table.insert / table.remove pairs in the generated code.
    std::vector<std::string> pf;  // pending float expression stack
    bool pending_negated = false; // logical-NOT applied to pf.back()
    bool pending_str = false;
    std::string pending_str_expr;

    void emit_push_float(std::string& out, int depth, const std::string& expr) {
        out += ind(depth) + "_pushf(" + expr + ")\n";
    }

    void emit_push_str(std::string& out, int depth, const std::string& expr) {
        out += ind(depth) + "_pushs(" + expr + ")\n";
    }

    static std::string pop_float_expr() {
        return "_popf()";
    }

    static std::string pop_str_expr() {
        return "_pops()";
    }

    static std::string peek_float_expr(int offset = 0) {
        if (offset == 0)
            return "_peekf()";
        return "_peekf(" + std::to_string(offset) + ")";
    }

    static std::string peek_str_expr(int offset = 0) {
        if (offset == 0)
            return "_peeks()";
        return "_peeks(" + std::to_string(offset) + ")";
    }

    // Materialise every pending expression onto the live stacks.
    void flush_pending(std::string& out, int depth) {
        for (size_t i = 0; i + 1 < pf.size(); i++)
            emit_push_float(out, depth, pf[i]);
        if (!pf.empty()) {
            if (pending_negated)
                emit_push_float(out, depth, "compare(" + pf.back() + ",0) and 1 or 0");
            else
                emit_push_float(out, depth, pf.back());
            pf.clear();
            pending_negated = false;
        }
        if (pending_str) {
            emit_push_str(out, depth, pending_str_expr);
            pending_str = false;
            pending_str_expr.clear();
        }
    }

    // Pop the top pending float expression (used by store-fold and if-fold).
    std::string take_pending_float() {
        std::string e = std::move(pf.back());
        pf.pop_back();
        pending_negated = false;
        return e;
    }
    std::string take_pending_str() {
        pending_str = false;
        std::string e = std::move(pending_str_expr);
        return e;
    }

    // Queue a float push onto the expression stack.
    // Materialises any pending negation on the current top first so the flag
    // always refers unambiguously to the new top.
    void push_float(std::string& /*out*/, int /*depth*/, const std::string& expr) {
        if (pending_negated && !pf.empty()) {
            pf.back() = "compare(" + pf.back() + ",0) and 1 or 0";
            pending_negated = false;
        }
        pf.push_back(expr);
    }
    // Queue a string push.  Flushes any previous pending string first.
    void push_str_val(std::string& out, int depth, const std::string& expr) {
        if (pending_str)
            emit_push_str(out, depth, pending_str_expr);
        pending_str = true;
        pending_str_expr = expr;
    }

    // ---- fold helpers -----------------------------------------------
    // Materialise any negation on pf.back() into the string itself.
    void materialise_neg() {
        if (pending_negated && !pf.empty()) {
            pf.back() = "compare(" + pf.back() + ",0) and 1 or 0";
            pending_negated = false;
        }
    }
    // Binary fold: infix operator (handles +, -, *, comparison, etc.)
    // Falls back to full flush + runtime emit when < 2 pending.
    void fold_infix(std::string& out, int depth, const std::string& infix,
                    const std::string& fallback) {
        if (pf.size() >= 2) {
            materialise_neg();
            std::string b = std::move(pf.back());
            pf.pop_back();
            pf.back() = "(" + pf.back() + infix + b + ")";
        } else {
            flush_pending(out, depth);
            out += fallback;
        }
    }

    void fold_comparison(std::string& out, int depth, const std::string& infix,
                         const std::string& fallback) {
        if (pf.size() >= 2) {
            materialise_neg();
            std::string b = std::move(pf.back());
            pf.pop_back();
            pf.back() = "((" + pf.back() + infix + b + ") and 1 or 0)";
        } else {
            flush_pending(out, depth);
            out += fallback;
        }
    }
    // Binary fold: two-argument function call (min, max, etc.)
    void fold_func2(std::string& out, int depth, const std::string& fn,
                    const std::string& fallback) {
        if (pf.size() >= 2) {
            materialise_neg();
            std::string b = std::move(pf.back());
            pf.pop_back();
            pf.back() = fn + "(" + pf.back() + "," + b + ")";
        } else {
            flush_pending(out, depth);
            out += fallback;
        }
    }
    // Unary fold: wrap top expression in a function call.
    void fold_unary_fn(std::string& out, int depth, const std::string& fn,
                       const std::string& fallback) {
        if (!pf.empty()) {
            materialise_neg();
            pf.back() = fn + "(" + pf.back() + ")";
        } else {
            flush_pending(out, depth);
            out += fallback;
        }
    }
    // Unary fold: arbitrary prefix/suffix wrap.
    void fold_unary_wrap(std::string& out, int depth, const std::string& pre,
                         const std::string& suf, const std::string& fallback) {
        if (!pf.empty()) {
            materialise_neg();
            pf.back() = pre + pf.back() + suf;
        } else {
            flush_pending(out, depth);
            out += fallback;
        }
    }

    Token& peek() {
        return toks[pos];
    }
    Token consume() {
        return toks[pos++];
    }

    // Consume tokens until the matching {end} is consumed, discarding everything.
    void skip_body() {
        int depth = 1;
        while (peek().type != TT::Eof) {
            Token t = consume();
            if (t.type == TT::BlockKW) {
                // Any block-opening keyword increments depth
                if (t.val == "if") {
                    ++depth;
                    continue;
                }
                if (t.val == "end") {
                    --depth;
                    if (depth == 0)
                        return;
                    continue;
                }
                if (t.val == "endif") {
                    --depth;
                    if (depth == 0)
                        return;
                    continue;
                }
                // macro:/trigger:/frame/etc nested inside another block also need tracking
                bool is_entry = t.val == "frame" || t.val == "frame_ai" || t.val == "init" ||
                                (t.val.size() > 6 && t.val.substr(0, 6) == "macro:") ||
                                (t.val.size() > 8 && t.val.substr(0, 8) == "trigger:");
                if (is_entry)
                    ++depth;
            }
        }
    }

    bool is_kw(const std::string& v) {
        return peek().type == TT::BlockKW && peek().val == v;
    }

    // ------------------------------------------------------------------ block
    void emit_block(std::string& out) {
        Token hdr = consume(); // opening keyword  e.g. {frame}
        std::cerr << "  block: {" << hdr.val << "} at line " << hdr.line << "\n";

        std::string mname;
        if (hdr.val.size() > 6 && hdr.val.substr(0, 6) == "macro:")
            mname = lower_name(hdr.val.substr(6));

        // 1. Builtin override: inline the pre-written Lua body
        if (!mname.empty() && builtin_overrides.count(mname)) {
            skip_body();
            std::string fname = func_name(hdr.val);
            out += "-- [builtin override] " + mname + "\n";
            out += "function " + fname + "()\n";
            out += builtin_overrides.at(mname);
            out += "end\n\n";
            return;
        }

        // 2. Runtime-dispatch override: emit a stub that calls a host Lua function
        if (!mname.empty() && overrides.count(mname)) {
            skip_body();
            std::string fname = func_name(hdr.val);
            out += "-- [override] " + mname + "\n";
            out += "function " + fname + "()\n";
            out += "  override_macro_" + sanitize(mname) + "(_fs, _ss)\n";
            out += "end\n\n";
            return;
        }

        pf.clear();
        pending_negated = false;
        pending_str = false;
        pending_str_expr.clear();
        out += "function " + func_name(hdr.val) + "()\n";
        emit_body(out, 1);
        out += "end\n\n";
    }

    // ------------------------------------------------------------------ body
    // Consumes until {end}, {endif}, or {else}  (does NOT consume the stopper)
    void emit_body(std::string& out, int depth) {
        while (true) {
            if (peek().type == TT::Eof) {
                std::cerr << "Warning: EOF inside body (missing {end}?) at line " << peek().line
                          << "\n";
                break;
            }
            if (is_kw("end")) {
                flush_pending(out, depth);
                consume();
                break;
            }
            if (is_kw("endif") || is_kw("else")) {
                flush_pending(out, depth);
                break;
            }
            size_t before = pos;
            emit_stmt(out, depth);
            if (pos == before) {
                std::cerr << "Warning: no progress at token '" << peek().val << "' line "
                          << peek().line << " - forcing advance\n";
                consume();
            }
        }
    }

    // ---------------------------------------------------------------- statement
    void emit_stmt(std::string& out, int depth) {
        Token& t = peek();

        if (t.type == TT::Comment) {
            flush_pending(out, depth);
            Token c = consume();
            std::string text = c.val;
            while (!text.empty() && std::isspace((unsigned char)text.front()))
                text.erase(text.begin());
            while (!text.empty() && std::isspace((unsigned char)text.back()))
                text.pop_back();
            out += ind(depth) + "--" + (text.empty() ? "" : " " + text) + "\n";
            return;
        }

        if (t.type == TT::Number) {
            Token n = consume();
            push_float(out, depth, n.val);
            return;
        }

        if (t.type == TT::String) {
            Token s = consume();
            push_str_val(out, depth, "\"" + lua_escape(s.val) + "\"");
            return;
        }

        if (t.type == TT::Command) {
            Token c = consume();
            emit_command(out, depth, c.val, c.line);
            return;
        }

        if (t.type == TT::Ident) {
            Token id = consume();
            emit_ident(out, depth, id.val, id.line);
            return;
        }

        if (t.type == TT::BlockKW) {
            if (t.val == "if") {
                emit_if(out, depth);
                return;
            }
            flush_pending(out, depth);
            std::cerr << "Warning: unexpected keyword {" << t.val << "} at line " << t.line << "\n";
            consume();
            return;
        }

        consume(); // skip anything else
    }

    // --------------------------------------------------------------- command
    void emit_command(std::string& out, int depth, const std::string& cmd, int ln) {
        // Format: X.Y.name   (minimum "L.L.x" = 5 chars)
        if (cmd.size() < 5 || cmd[1] != '.' || cmd[3] != '.') {
            out += ind(depth) + "-- UNKNOWN COMMAND: (" + cmd + ")  [line " + std::to_string(ln) +
                   "]\n";
            return;
        }

        char type = (char)std::toupper((unsigned char)cmd[0]);
        char sub = cmd[2]; // keep original case, e.g. '$'
        std::string name = cmd.substr(4);

        // Normalise L/S/M/T/C/F sub-type letters to uppercase for matching
        char subU = (char)std::toupper((unsigned char)sub);

        switch (type) {

        case 'L': // Load
            if (subU == 'L')
                push_float(out, depth, "get_local_var(\"" + lower_name(name) + "\")");
            else if (subU == 'S')
                push_float(out, depth, "get_sys_var(\"" + lower_name(name) + "\")");
            else if (sub == '$')
                push_str_val(out, depth, "get_local_str(\"" + lower_name(name) + "\")");
            else {
                flush_pending(out, depth);
                out += ind(depth) + "-- UNKNOWN LOAD sub-type '" + std::string(1, sub) + "': (" +
                       cmd + ")\n";
            }
            break;

        case 'S': // Store
            if (subU == 'L') {
                const std::string normalizedName = lower_name(name);
                if (!pf.empty()) {
                    std::string expr =
                        pending_negated ? "compare(" + pf.back() + ",0) and 1 or 0" : pf.back();
                    out += ind(depth) + "set_local_var(\"" + normalizedName + "\", " + expr + ")\n";
                } else {
                    flush_pending(out, depth);
                    out += ind(depth) + "set_local_var(\"" + normalizedName + "\", " +
                           peek_float_expr() + ")\n";
                }
            } else if (sub == '$') {
                const std::string normalizedName = lower_name(name);
                if (pending_str) {
                    out += ind(depth) + "set_local_str(\"" + normalizedName + "\", " +
                           pending_str_expr + ")\n";
                } else {
                    flush_pending(out, depth);
                    out += ind(depth) + "set_local_str(\"" + normalizedName + "\", " +
                           peek_str_expr() + ")\n";
                }
            } else {
                flush_pending(out, depth);
                out += ind(depth) + "-- UNKNOWN STORE sub-type '" + std::string(1, sub) + "': (" +
                       cmd + ")\n";
            }
            break;

        case 'M': // Macro call
            flush_pending(out, depth);
            if (subU == 'L')
                out += ind(depth) + "macro_" + sanitize(name) + "()\n";
            else if (subU == 'V')
                out += ind(depth) + "sys_macro_" + sanitize(name) + "()\n";
            else
                out += ind(depth) + "-- UNKNOWN MACRO sub-type: (" + cmd + ")\n";
            break;

        case 'T': // Sound trigger
            flush_pending(out, depth);
            if (subU == 'L')
                out += ind(depth) + "sound_trigger(\"" + lower_name(name) + "\")\n";
            else if (subU == 'F') {
                if (pending_str)
                    out += ind(depth) + "sound_trigger_file(\"" + lower_name(name) + "\", " +
                           take_pending_str() + ")\n";
                else {
                    flush_pending(out, depth);
                    out += ind(depth) + "sound_trigger_file(\"" + lower_name(name) + "\", " +
                           pop_str_expr() + ")\n";
                }
            } else
                out += ind(depth) + "-- UNKNOWN TRIGGER sub-type: (" + cmd + ")\n";
            break;

        case 'C': // Constant load
            push_float(out, depth, "get_const(\"" + lower_name(name) + "\")");
            break;

        case 'F': // Function/curve call  (pops x, pushes y)
            if (!pf.empty()) {
                materialise_neg();
                pf.back() = "call_func(\"" + lower_name(name) + "\", " + pf.back() + ")";
            } else {
                flush_pending(out, depth);
                emit_push_float(out, depth,
                                "call_func(\"" + lower_name(name) + "\", " + pop_float_expr() +
                                    ")");
            }
            break;

        default:
            flush_pending(out, depth);
            out += ind(depth) + "-- UNKNOWN COMMAND type '" + std::string(1, type) + "': (" + cmd +
                   ")\n";
        }
    }

    // --------------------------------------------------------------- operator
    void emit_ident(std::string& out, int depth, const std::string& op, int ln) {
        // All operators interact with the live stack, so flush any pending push first.
        // Exception: 'pi' and 'l0'-'l7' are pure pushes - handle them via push_float.
        // We handle those before the flush below.

        // ---- registers  l0-l7  (push register value) - treated as a push
        if (op.size() == 2 && op[0] == 'l' && op[1] >= '0' && op[1] <= '7') {
            int r = op[1] - '0';
            push_float(out, depth, "_r" + std::to_string(r));
            return;
        }
        if (op == "pi") {
            push_float(out, depth, "math.pi");
            return;
        }

        // ---- logical NOT:  if a pending float exists just flip the negation flag
        //      so that a following {if} can fold to  "if expr == 0 then"
        if (op == "!") {
            if (!pf.empty()) {
                pending_negated = !pending_negated;
            } else {
                flush_pending(out, depth);
                emit_push_float(out, depth, "(compare(" + pop_float_expr() + ",0) and 1 or 0)");
            }
            return;
        }

        // ---- registers  s0-s7  (peek top; must flush so _fs is live)
        if (op.size() == 2 && op[0] == 's' && op[1] >= '0' && op[1] <= '7') {
            flush_pending(out, depth);
            int r = op[1] - '0';
            out += ind(depth) + "_r" + std::to_string(r) + "=" + peek_float_expr() + "\n";
            return;
        }

        // ---- arithmetic (fold when 2+ pending, else runtime)
        if (op == "+") {
            fold_infix(out, depth, "+",
                       ind(depth) + "do local b=" + pop_float_expr() +
                           "; local a=" + pop_float_expr() + "; _pushf(a+b) end\n");
            return;
        }
        if (op == "-") {
            fold_infix(out, depth, "-",
                       ind(depth) + "do local b=" + pop_float_expr() +
                           "; local a=" + pop_float_expr() + "; _pushf(a-b) end\n");
            return;
        }
        if (op == "*") {
            fold_infix(out, depth, "*",
                       ind(depth) + "do local b=" + pop_float_expr() +
                           "; local a=" + pop_float_expr() + "; _pushf(a*b) end\n");
            return;
        }
        if (op == "/") {
            if (pf.size() >= 2) {
                materialise_neg();
                std::string b = std::move(pf.back());
                pf.pop_back();
                pf.back() = "(compare(" + b + ",1) and " + pf.back() + "/" + b + " or 0)";
            } else {
                flush_pending(out, depth);
                out += ind(depth) + "do local b=" + pop_float_expr() +
                       "; local a=" + pop_float_expr() +
                       "; _pushf((compare(b,1) and a/b or 0)) end\n";
            }
            return;
        }
        if (op == "%") {
            if (pf.size() >= 2) {
                materialise_neg();
                std::string b = std::move(pf.back());
                pf.pop_back();
                const std::string a = pf.back();
                pf.back() = "(compare(" + b + ",1) and (" + a + "-math.floor(" + a + "/" + b +
                            ")*" + b + ") or 0)";
            } else {
                flush_pending(out, depth);
                out += ind(depth) + "do local b=" + pop_float_expr() +
                       "; local a=" + pop_float_expr() + "\n";
                out += ind(depth) + "  _pushf((compare(b,1) and (a-math.floor(a/b)*b) or 0)) end\n";
            }
            return;
        }
        if (op == "/-/") {
            fold_unary_wrap(out, depth, "-(", ")",
                            ind(depth) + "_pushf(-" + pop_float_expr() + ")\n");
            return;
        }

        // ---- math functions (unary, fold or in-place)
        if (op == "sin") {
            fold_unary_fn(out, depth, "math.sin",
                          ind(depth) + "_pushf(math.sin(" + pop_float_expr() + "))\n");
            return;
        }
        if (op == "arcsin") {
            fold_unary_fn(out, depth, "math.asin",
                          ind(depth) + "_pushf(math.asin(" + pop_float_expr() + "))\n");
            return;
        }
        if (op == "arctan") {
            fold_unary_fn(out, depth, "math.atan",
                          ind(depth) + "_pushf(math.atan(" + pop_float_expr() + "))\n");
            return;
        }
        if (op == "exp") {
            fold_unary_fn(out, depth, "math.exp",
                          ind(depth) + "_pushf(math.exp(" + pop_float_expr() + "))\n");
            return;
        }
        if (op == "sqrt") {
            fold_unary_wrap(out, depth, "math.sqrt(math.max(0,", "))",
                            ind(depth) + "_pushf(math.sqrt(math.max(0," + pop_float_expr() +
                                ")))\n");
            return;
        }
        if (op == "abs") {
            fold_unary_fn(out, depth, "math.abs",
                          ind(depth) + "_pushf(math.abs(" + pop_float_expr() + "))\n");
            return;
        }
        if (op == "trunc") {
            fold_unary_fn(out, depth, "math.floor",
                          ind(depth) + "_pushf(math.floor(" + pop_float_expr() + "))\n");
            return;
        }
        if (op == "sqr") {
            if (!pf.empty()) {
                materialise_neg();
                const std::string e = pf.back();
                pf.back() = "(" + e + "*" + e + ")";
            } else {
                flush_pending(out, depth);
                out += ind(depth) + "do local v=" + pop_float_expr() + "; _pushf(v*v) end\n";
            }
            return;
        }
        if (op == "sgn") {
            if (!pf.empty()) {
                materialise_neg();
                const std::string e = pf.back();
                pf.back() = "((" + e + ">0) and 1 or ((" + e + "<0) and -1 or 0))";
            } else {
                flush_pending(out, depth);
                out += ind(depth) + "do local v=" + pop_float_expr() +
                       "; _pushf((v>0 and 1 or (v<0 and -1 or 0))) end\n";
            }
            return;
        }
        if (op == "random") {
            if (!pf.empty()) {
                materialise_neg();
                const std::string e = pf.back();
                pf.back() = "math.random(0,math.max(0,math.floor(" + e + ")-1))";
            } else {
                flush_pending(out, depth);
                out += ind(depth) + "_pushf(math.random(0,math.max(0,math.floor(" +
                       pop_float_expr() + ")-1)))\n";
            }
            return;
        }

        // ---- binary math
        if (op == "min") {
            fold_func2(out, depth, "math.min",
                       ind(depth) + "do local b=" + pop_float_expr() +
                           "; local a=" + pop_float_expr() + "; _pushf(math.min(a,b)) end\n");
            return;
        }
        if (op == "max") {
            fold_func2(out, depth, "math.max",
                       ind(depth) + "do local b=" + pop_float_expr() +
                           "; local a=" + pop_float_expr() + "; _pushf(math.max(a,b)) end\n");
            return;
        }

        // ---- logical
        if (op == "&&") {
            if (pf.size() >= 2) {
                materialise_neg();
                std::string b = std::move(pf.back());
                pf.pop_back();
                pf.back() = "((compare(" + pf.back() + ",1) and compare(" + b + ",1)) and 1 or 0)";
            } else {
                flush_pending(out, depth);
                out += ind(depth) + "do local b=" + pop_float_expr() +
                       "; local a=" + pop_float_expr() +
                       "; _pushf((compare(a,1) and compare(b,1)) and 1 or 0) end\n";
            }
            return;
        }
        if (op == "||") {
            if (pf.size() >= 2) {
                materialise_neg();
                std::string b = std::move(pf.back());
                pf.pop_back();
                pf.back() = "((compare(" + pf.back() + ",1) or compare(" + b + ",1)) and 1 or 0)";
            } else {
                flush_pending(out, depth);
                out += ind(depth) + "do local b=" + pop_float_expr() +
                       "; local a=" + pop_float_expr() +
                       "; _pushf((compare(a,1) or compare(b,1)) and 1 or 0) end\n";
            }
            return;
        }
        // ---- numeric comparison (a = stack1, b = stack0)
        if (op == "=") {
            fold_comparison(out, depth, "==",
                            ind(depth) + "do local b=" + pop_float_expr() + "; local a=" +
                                pop_float_expr() + "; _pushf(a==b and 1 or 0) end\n");
            return;
        }
        if (op == "<") {
            fold_comparison(out, depth, "<",
                            ind(depth) + "do local b=" + pop_float_expr() +
                                "; local a=" + pop_float_expr() + "; _pushf(a<b and 1 or 0) end\n");
            return;
        }
        if (op == ">") {
            fold_comparison(out, depth, ">",
                            ind(depth) + "do local b=" + pop_float_expr() +
                                "; local a=" + pop_float_expr() + "; _pushf(a>b and 1 or 0) end\n");
            return;
        }
        if (op == "<=") {
            fold_comparison(out, depth, "<=",
                            ind(depth) + "do local b=" + pop_float_expr() + "; local a=" +
                                pop_float_expr() + "; _pushf(a<=b and 1 or 0) end\n");
            return;
        }
        if (op == ">=") {
            fold_comparison(out, depth, ">=",
                            ind(depth) + "do local b=" + pop_float_expr() + "; local a=" +
                                pop_float_expr() + "; _pushf(a>=b and 1 or 0) end\n");
            return;
        }

        // ---- float stack misc
        if (op == "d") {
            flush_pending(out, depth); // must materialise before duplicating
            out += ind(depth) + "_pushf(" + peek_float_expr() + ")\n";
            return;
        }

        // ---- string stack ops
        if (op == "$d") {
            flush_pending(out, depth);
            out += ind(depth) + "_pushs(" + peek_str_expr() + ")\n";
            return;
        }
        if (op == "$msg") {
            flush_pending(out, depth);
            out += ind(depth) + "omsi_debug(" + peek_str_expr() + ")\n";
            return;
        }
        if (op == "$+") {
            flush_pending(out, depth);
            out += ind(depth) + "do local b=" + pop_str_expr() + "; local a=" + pop_str_expr() +
                   "; _pushs(a..b) end\n";
            return;
        }
        if (op == "$*") {
            // repeat string s until total length would exceed n  (pops n from _fs, modifies top of
            // _ss)
            flush_pending(out, depth);
            out += ind(depth) + "do\n";
            out += ind(depth) + "  local n=math.floor(" + pop_float_expr() + ")\n";
            out += ind(depth) + "  local s=" + pop_str_expr() + "\n";
            out += ind(depth) + "  local res=\"\"\n";
            out += ind(depth) + "  if #s>0 then while #res+#s<=n do res=res..s end end\n";
            out += ind(depth) + "  _pushs(res)\n";
            out += ind(depth) + "end\n";
            return;
        }
        if (op == "$length") {
            flush_pending(out, depth);
            out += ind(depth) + "_pushf(#" + peek_str_expr() + ")\n";
            return;
        }
        if (op == "$cutBegin") {
            flush_pending(out, depth);
            out += ind(depth) + "do local n=math.floor(" + pop_float_expr() +
                   "); local s=" + pop_str_expr() + "; _pushs(string.sub(s,n+1)) end\n";
            return;
        }
        if (op == "$cutEnd") {
            flush_pending(out, depth);
            out += ind(depth) + "do local n=math.floor(" + pop_float_expr() +
                   "); local s=" + pop_str_expr() + "\n";
            out += ind(depth) + "  _pushs((n>0 and string.sub(s,1,#s-n) or s)) end\n";
            return;
        }
        // $SetLengthL/R/C  NOTE: does NOT pop the float from the stack (per spec)
        if (op == "$SetLengthL") {
            flush_pending(out, depth);
            out +=
                ind(depth) + "do local n=math.floor(" + peek_float_expr() + "); local s=_pops()\n";
            out += ind(depth) + "  if     #s>n then _pushs(string.sub(s,1,n))\n";
            out += ind(depth) + "  elseif #s<n then _pushs(s..string.rep(\" \",n-#s))\n";
            out += ind(depth) + "  else _pushs(s) end end\n";
            return;
        }
        if (op == "$SetLengthR") {
            flush_pending(out, depth);
            out +=
                ind(depth) + "do local n=math.floor(" + peek_float_expr() + "); local s=_pops()\n";
            out += ind(depth) + "  if     #s>n then _pushs(string.sub(s,#s-n+1))\n";
            out += ind(depth) + "  elseif #s<n then _pushs(string.rep(\" \",n-#s)..s)\n";
            out += ind(depth) + "  else _pushs(s) end end\n";
            return;
        }
        if (op == "$SetLengthC") {
            flush_pending(out, depth);
            out +=
                ind(depth) + "do local n=math.floor(" + peek_float_expr() + "); local s=_pops()\n";
            out += ind(depth) + "  if #s>n then\n";
            out += ind(depth) + "    local cut=#s-n; local l=math.floor(cut/2)\n";
            out += ind(depth) + "    _pushs(string.sub(s,l+1,l+n))\n";
            out += ind(depth) + "  elseif #s<n then\n";
            out += ind(depth) + "    local pad=n-#s; local l=math.floor(pad/2)\n";
            out += ind(depth) + "    _pushs(string.rep(\" \",l)..s..string.rep(\" \",pad-l))\n";
            out += ind(depth) + "  else _pushs(s) end end\n";
            return;
        }
        if (op == "$IntToStr") {
            // pushes int representation of top float onto string stack; float stays
            flush_pending(out, depth);
            out += ind(depth) + "_pushs(tostring(math.floor(" + peek_float_expr() + ")))\n";
            return;
        }
        if (op == "$IntToStrEnh") {
            // format string on _ss, value on _fs (both popped)
            flush_pending(out, depth);
            out += ind(depth) + "do\n";
            out += ind(depth) + "  local fmt=" + pop_str_expr() + "\n";
            out += ind(depth) + "  local v=math.floor(" + pop_float_expr() + ")\n";
            out += ind(depth) + "  if #fmt<2 then _pushs(\"ERROR\")\n";
            out += ind(depth) + "  else\n";
            out += ind(depth) + "    local fill=string.sub(fmt,1,1)\n";
            out += ind(depth) + "    local digits=tonumber(string.sub(fmt,2))\n";
            out += ind(depth) + "    if digits==nil then _pushs(\"ERROR\")\n";
            out += ind(depth) + "    else\n";
            out += ind(depth) + "      local s=tostring(v)\n";
            out += ind(depth) + "      if #s<digits then s=string.rep(fill,digits-#s)..s end\n";
            out += ind(depth) + "      _pushs(s)\n";
            out += ind(depth) + "    end\n";
            out += ind(depth) + "  end\n";
            out += ind(depth) + "end\n";
            return;
        }
        if (op == "$StrToFloat") {
            flush_pending(out, depth);
            out += ind(depth) + "_pushf(tonumber(" + pop_str_expr() + ") or -1)\n";
            return;
        }
        if (op == "$RemoveSpaces") {
            flush_pending(out, depth);
            out += ind(depth) +
                   "do local s=_pops(); _pushs(s:gsub(\"^%s+\",\"\"):gsub(\"%s+$\",\"\")) end\n";
            return;
        }

        // ---- string comparison (pop both strings, push 0/1 onto float stack)
        if (op == "$=") {
            flush_pending(out, depth);
            out += ind(depth) + "do local b=" + pop_str_expr() + "; local a=" + pop_str_expr() +
                   "; _pushf(a==b and 1 or 0) end\n";
            return;
        }
        if (op == "$<") {
            flush_pending(out, depth);
            out += ind(depth) + "do local b=" + pop_str_expr() + "; local a=" + pop_str_expr() +
                   "; _pushf(a<b and 1 or 0) end\n";
            return;
        }
        if (op == "$>") {
            flush_pending(out, depth);
            out += ind(depth) + "do local b=" + pop_str_expr() + "; local a=" + pop_str_expr() +
                   "; _pushf(a>b and 1 or 0) end\n";
            return;
        }
        if (op == "$<=") {
            flush_pending(out, depth);
            out += ind(depth) + "do local b=" + pop_str_expr() + "; local a=" + pop_str_expr() +
                   "; _pushf(a<=b and 1 or 0) end\n";
            return;
        }
        if (op == "$>=") {
            flush_pending(out, depth);
            out += ind(depth) + "do local b=" + pop_str_expr() + "; local a=" + pop_str_expr() +
                   "; _pushf(a>=b and 1 or 0) end\n";
            return;
        }

        if (op == "%stackdump%") {
            out += ind(depth) + "-- %stackdump% (debug)\n";
            return;
        }

        out += ind(depth) + "-- UNKNOWN OP: " + op + "  [line " + std::to_string(ln) + "]\n";
    }

    // --------------------------------------------------------------- if/else/endif
    void emit_if(std::string& out, int depth) {
        consume(); // consume {if}
        if (!pf.empty()) {
            bool negated = pending_negated;
            std::string expr = take_pending_float();
            const std::string condition = "_osc_condition";
            out += ind(depth) + "local " + condition + "=" + expr + "\n";
            if (negated) {
                out += ind(depth) + "if not compare(" + condition + ",1) then\n";
            } else {
                out += ind(depth) + "if compare(" + condition + ",1) then\n";
            }
        } else {
            flush_pending(out, depth);
            const std::string expr = pop_float_expr();
            out += ind(depth) + "local _osc_condition=" + expr + "\n";
            out += ind(depth) + "if compare(_osc_condition,1) then\n";
        }
        emit_body(out, depth + 1);

        if (is_kw("else")) {
            consume();
            out += ind(depth) + "else\n";
            emit_body(out, depth + 1);
        }

        if (is_kw("endif"))
            consume();
        out += ind(depth) + "end\n";
    }

  public:
    explicit Emitter(std::vector<Token> t)
        : toks(std::move(t)), builtin_overrides(get_builtin_overrides()) {}

    void add_override(const std::string& macro_name) {
        overrides.insert(lower_name(macro_name));
    }

    std::string emit() {
        std::string out;

        out += "-- Generated by osc2lua\n";
        // out += "-- Stack layout: _fs/_ss are shared stacks driven by push/pop helpers.\n";
        // out += "local _fs = {}\n";
        // out += "local _ss = {}\n";
        // out += "local function _pushf(v) _fs[#_fs + 1] = v end\n";
        // out += "local function _popf() local i = #_fs; local v = _fs[i]; _fs[i] = nil; return v
        // end\n"; out += "local function _peekf(offset) offset = offset or 0; return _fs[#_fs -
        // offset] end\n"; out += "local function _pushs(v) _ss[#_ss + 1] = v end\n"; out += "local
        // function _pops() local i = #_ss; local v = _ss[i]; _ss[i] = nil; return v end\n"; out +=
        // "local function _peeks(offset) offset = offset or 0; return _ss[#_ss - offset] end\n";
        out += "\n";
        out += "-- Host must provide: get_local_var, set_local_var, get_sys_var,\n";
        out += "--   get_local_str, set_local_str, get_const, call_func,\n";
        out += "--   sound_trigger, sound_trigger_file, sys_macro_*, omsi_debug\n";
        out += "\n";
        out += "local function compare(a,b)\n";
        out += "  if type(a)==\"boolean\" then return (a and 1 or 0)==b end\n";
        out += "  return (a~=0)==(b~=0)\n";
        out += "end\n";
        out += "\n";
        out += "-- Shared registers  (s0-s7 / l0-l7)\n";
        out += "local _r0,_r1,_r2,_r3,_r4,_r5,_r6,_r7 = 0,0,0,0,0,0,0,0\n";
        out += "\n";
        while (peek().type != TT::Eof) {
            Token& t = peek();

            if (t.type == TT::Comment) {
                Token c = consume();
                std::string text = c.val;
                while (!text.empty() && std::isspace((unsigned char)text.front()))
                    text.erase(text.begin());
                while (!text.empty() && std::isspace((unsigned char)text.back()))
                    text.pop_back();
                out += "--" + (text.empty() ? "" : " " + text) + "\n";
                continue;
            }

            if (t.type == TT::BlockKW) {
                const std::string& kw = t.val;
                bool is_entry = kw == "frame" || kw == "frame_ai" || kw == "init" ||
                                (kw.size() > 6 && kw.substr(0, 6) == "macro:") ||
                                (kw.size() > 8 && kw.substr(0, 8) == "trigger:");
                if (is_entry) {
                    emit_block(out);
                    continue;
                }
                // stray keyword at top level
                std::cerr << "Warning: stray keyword {" << kw << "} at line " << t.line << "\n";
                consume();
                continue;
            }

            // stray statement at top level (shouldn't normally happen)
            std::cerr << "Warning: stray token '" << t.val << "' at line " << t.line << "\n";
            consume();
        }

        return out;
    }
};

class BytecodeCompiler {
    std::vector<Token> tokens_;
    std::size_t position_ = 0;
    OscProgram& output_;
    std::string error_;

    Token& peek() {
        return tokens_[position_];
    }

    Token consume() {
        return tokens_[position_++];
    }

    void unsupported(const Token& token, const std::string& detail) {
        if (error_.empty()) {
            error_ = detail + " at line " + std::to_string(token.line);
        }
    }

    static void emit(std::vector<OscInstruction>& code, OscOpcode opcode) {
        code.push_back({opcode, 0.0, 0, {}});
    }

    static void emitName(std::vector<OscInstruction>& code, OscOpcode opcode,
                         const std::string& name) {
        code.push_back({opcode, 0.0, 0, name});
    }

    static std::size_t emitJump(std::vector<OscInstruction>& code, OscOpcode opcode) {
        code.push_back({opcode, 0.0, 0, {}});
        return code.size() - 1;
    }

    void compileCommand(const Token& token, std::vector<OscInstruction>& code) {
        const std::string& command = token.val;
        if (command.size() < 5 || command[1] != '.' || command[3] != '.') {
            unsupported(token, "unsupported OSC command (" + command + ")");
            return;
        }

        const char type = static_cast<char>(std::toupper(static_cast<unsigned char>(command[0])));
        const char sub = command[2];
        const char subUpper = static_cast<char>(std::toupper(static_cast<unsigned char>(sub)));
        const std::string name = lower_name(command.substr(4));

        if (type == 'L' && subUpper == 'L') {
            emitName(code, OscOpcode::LoadLocal, name);
        } else if (type == 'L' && subUpper == 'S') {
            emitName(code, OscOpcode::LoadSystem, name);
        } else if (type == 'L' && sub == '$') {
            emitName(code, OscOpcode::LoadLocalString, name);
        } else if (type == 'S' && subUpper == 'L') {
            emitName(code, OscOpcode::StoreLocal, name);
        } else if (type == 'S' && sub == '$') {
            emitName(code, OscOpcode::StoreLocalString, name);
        } else if (type == 'C') {
            emitName(code, OscOpcode::LoadConstant, name);
        } else if (type == 'F') {
            emitName(code, OscOpcode::CallCurve, name);
        } else if (type == 'M' && subUpper == 'L') {
            emitName(code, OscOpcode::CallFunction, "macro_" + sanitize(name));
        } else if (type == 'M' && subUpper == 'V') {
            emitName(code, OscOpcode::CallSystemMacro, sanitize(name));
        } else if (type == 'T' && subUpper == 'L') {
            emitName(code, OscOpcode::SoundTrigger, name);
        } else if (type == 'T' && subUpper == 'F') {
            emitName(code, OscOpcode::SoundTriggerFile, name);
        } else {
            unsupported(token, "unsupported OSC command (" + command + ")");
        }
    }

    void compileIdentifier(const Token& token, std::vector<OscInstruction>& code) {
        const std::string& op = token.val;
        if (op.size() == 2 && op[0] == 'l' && op[1] >= '0' && op[1] <= '7') {
            code.push_back({OscOpcode::LoadRegister, 0.0, op[1] - '0', {}});
        } else if (op.size() == 2 && op[0] == 's' && op[1] >= '0' && op[1] <= '7') {
            code.push_back({OscOpcode::StoreRegister, 0.0, op[1] - '0', {}});
        } else if (op == "pi") {
            code.push_back({OscOpcode::PushNumber, 3.14159265358979323846, 0, {}});
        } else if (op == "d") {
            emit(code, OscOpcode::Duplicate);
        } else if (op == "+") {
            emit(code, OscOpcode::Add);
        } else if (op == "-") {
            emit(code, OscOpcode::Subtract);
        } else if (op == "*") {
            emit(code, OscOpcode::Multiply);
        } else if (op == "/") {
            emit(code, OscOpcode::Divide);
        } else if (op == "%") {
            emit(code, OscOpcode::Modulo);
        } else if (op == "/-/") {
            emit(code, OscOpcode::Negate);
        } else if (op == "!") {
            emit(code, OscOpcode::LogicalNot);
        } else if (op == "=") {
            emit(code, OscOpcode::Equal);
        } else if (op == "!=") {
            emit(code, OscOpcode::NotEqual);
        } else if (op == "<") {
            emit(code, OscOpcode::Less);
        } else if (op == ">") {
            emit(code, OscOpcode::Greater);
        } else if (op == "<=") {
            emit(code, OscOpcode::LessEqual);
        } else if (op == ">=") {
            emit(code, OscOpcode::GreaterEqual);
        } else if (op == "&&") {
            emit(code, OscOpcode::LogicalAnd);
        } else if (op == "||") {
            emit(code, OscOpcode::LogicalOr);
        } else if (op == "abs") {
            emit(code, OscOpcode::Absolute);
        } else if (op == "min") {
            emit(code, OscOpcode::Minimum);
        } else if (op == "max") {
            emit(code, OscOpcode::Maximum);
        } else if (op == "trunc") {
            emit(code, OscOpcode::Floor);
        } else if (op == "sqrt") {
            emit(code, OscOpcode::SquareRoot);
        } else if (op == "arcsin") {
            emit(code, OscOpcode::ArcSine);
        } else if (op == "exp") {
            emit(code, OscOpcode::Exponential);
        } else if (op == "sin") {
            emit(code, OscOpcode::Sine);
        } else if (op == "cos") {
            emit(code, OscOpcode::Cosine);
        } else if (op == "tan") {
            emit(code, OscOpcode::Tangent);
        } else if (op == "arctan") {
            emit(code, OscOpcode::ArcTangent);
        } else if (op == "ceiling") {
            emit(code, OscOpcode::Ceiling);
        } else if (op == "sqr") {
            emit(code, OscOpcode::Square);
        } else if (op == "sgn") {
            emit(code, OscOpcode::Sign);
        } else if (op == "random") {
            emit(code, OscOpcode::Random);
        } else if (op == "$d") {
            emit(code, OscOpcode::StringDuplicate);
        } else if (op == "$+") {
            emit(code, OscOpcode::StringConcat);
        } else if (op == "$*") {
            emit(code, OscOpcode::StringRepeat);
        } else if (op == "$length") {
            emit(code, OscOpcode::StringLength);
        } else if (op == "$cutBegin") {
            emit(code, OscOpcode::StringCutBegin);
        } else if (op == "$cutEnd") {
            emit(code, OscOpcode::StringCutEnd);
        } else if (op == "$SetLengthL") {
            emit(code, OscOpcode::StringSetLengthLeft);
        } else if (op == "$SetLengthR") {
            emit(code, OscOpcode::StringSetLengthRight);
        } else if (op == "$SetLengthC") {
            emit(code, OscOpcode::StringSetLengthCenter);
        } else if (op == "$IntToStr") {
            emit(code, OscOpcode::IntegerToString);
        } else if (op == "$IntToStrEnh") {
            emit(code, OscOpcode::IntegerToStringEnhanced);
        } else if (op == "$StrToFloat") {
            emit(code, OscOpcode::StringToFloat);
        } else if (op == "$RemoveSpaces") {
            emit(code, OscOpcode::RemoveSpaces);
        } else if (op == "$=") {
            emit(code, OscOpcode::StringEqual);
        } else if (op == "$<") {
            emit(code, OscOpcode::StringLess);
        } else if (op == "$>") {
            emit(code, OscOpcode::StringGreater);
        } else if (op == "$<=") {
            emit(code, OscOpcode::StringLessEqual);
        } else if (op == "$>=") {
            emit(code, OscOpcode::StringGreaterEqual);
        } else if (op == "$") {
            emit(code, OscOpcode::StringNoOp);
        } else if (op == "$msg") {
            emit(code, OscOpcode::DebugString);
        } else if (op == "%stackdump%") {
            emit(code, OscOpcode::StackDump);
        } else {
            unsupported(token, "unsupported OSC operator " + op);
        }
    }

    bool is(const char* keyword) const {
        return tokens_[position_].type == TT::BlockKW && tokens_[position_].val == keyword;
    }

    void compileBody(std::vector<OscInstruction>& code) {
        while (!error_.empty() == false && peek().type != TT::Eof) {
            if (is("end") || is("else") || is("endif")) {
                return;
            }
            const Token token = consume();
            if (token.type == TT::Comment) {
                continue;
            }
            if (token.type == TT::Number) {
                code.push_back({OscOpcode::PushNumber, std::stod(token.val), 0, {}});
            } else if (token.type == TT::String) {
                code.push_back({OscOpcode::PushString, 0.0, 0, token.val});
            } else if (token.type == TT::Command) {
                compileCommand(token, code);
            } else if (token.type == TT::Ident) {
                compileIdentifier(token, code);
            } else if (token.type == TT::BlockKW && token.val == "if") {
                const std::size_t falseJump = emitJump(code, OscOpcode::JumpIfFalse);
                compileBody(code);
                if (is("else")) {
                    consume();
                    const std::size_t endJump = emitJump(code, OscOpcode::Jump);
                    code[falseJump].index = static_cast<int>(code.size());
                    compileBody(code);
                    if (is("endif")) {
                        consume();
                    } else {
                        unsupported(token, "missing {endif}");
                    }
                    code[endJump].index = static_cast<int>(code.size());
                } else if (is("endif")) {
                    consume();
                    code[falseJump].index = static_cast<int>(code.size());
                } else {
                    unsupported(token, "missing {endif}");
                }
            } else {
                unsupported(token, "unsupported OSC token");
            }
        }
    }

    void compileBlock(const Token& header) {
        std::vector<OscInstruction> code;
        compileBody(code);
        if (is("end")) {
            consume();
        } else if (is("endif")) {
            const Token strayEndif = consume();
            while (is("endif"))
                consume();
            if (is("end"))
                consume();
            std::cerr << "Warning: treating stray {endif} as end of {" << header.val << "} at line "
                      << strayEndif.line << "\n";
        } else if (peek().type == TT::Eof) {
            std::cerr << "Warning: EOF inside {" << header.val
                      << "}; treating it as an implicit {end}\n";
        } else if (error_.empty()) {
            unsupported(header, "missing {end}");
        }
        output_.functions[func_name(header.val)] = std::move(code);
    }

  public:
    BytecodeCompiler(std::vector<Token> tokens, OscProgram& output)
        : tokens_(std::move(tokens)), output_(output) {}

    bool compile(std::string& error) {
        while (peek().type != TT::Eof && error_.empty()) {
            const Token token = consume();
            if (token.type == TT::Comment) {
                continue;
            }
            if (token.type == TT::BlockKW &&
                (token.val == "frame" || token.val == "frame_ai" || token.val == "init" ||
                 (token.val.size() > 6 && token.val.substr(0, 6) == "macro:") ||
                 (token.val.size() > 8 && token.val.substr(0, 8) == "trigger:"))) {
                compileBlock(token);
            } else {
                unsupported(token, "unsupported top-level OSC content");
            }
        }
        if (!error_.empty()) {
            error = error_;
            return false;
        }
        return true;
    }
};

// ============================================================
// MAIN
// ============================================================

static std::string read_file(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f)
        throw std::runtime_error("Cannot open file: " + p.string());
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static std::string cache_hash(const std::string& source) {
    std::uint64_t hash = 14695981039346656037ull;
    const std::string versionedSource = source + "\n" + kOscConverterVersion;
    for (unsigned char byte : versionedSource) {
        hash ^= byte;
        hash *= 1099511628211ull;
    }

    std::ostringstream formatted;
    formatted << std::hex << std::setw(16) << std::setfill('0') << hash;
    return formatted.str();
}

std::filesystem::path generatedLuaPath(const std::filesystem::path& inputPath) {
    const std::string source = read_file(inputPath);
    return inputPath.parent_path() /
           (inputPath.stem().string() + "." + cache_hash(source) + ".lua");
}

bool convertOscToLua(const std::filesystem::path& inputPath,
                     const std::filesystem::path& outputPath, std::string& error) {
    try {
        Tokenizer tokenizer(read_file(inputPath));
        Emitter emitter(tokenizer.tokenize());
        const std::string lua = emitter.emit();

        std::ofstream output(outputPath, std::ios::binary);
        if (!output) {
            error = "cannot write " + outputPath.string();
            return false;
        }
        output << lua;
        if (!output) {
            error = "failed while writing " + outputPath.string();
            return false;
        }
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

bool compileOscToBytecode(const std::filesystem::path& inputPath, OscProgram& output,
                          std::string& error) {
    try {
        output.functions.clear();
        Tokenizer tokenizer(read_file(inputPath));
        BytecodeCompiler compiler(tokenizer.tokenize(), output);
        return compiler.compile(error);
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

// int main(int argc, char* argv[]) {
//     if (argc < 2) {
//         std::cerr << "Usage: osc2lua <input.osc> [output.lua] [--override MacroName ...]\n";
//         return 1;
//     }

//     auto t_start = std::chrono::high_resolution_clock::now();

//     // Parse arguments:
//     //   argv[1]           = input file
//     //   argv[2]           = output file (optional, skipped if it starts with '--')
//     //   --override Name   = mark macro Name as overridden (repeatable)
//     std::filesystem::path inp  = argv[1];
//     std::filesystem::path outp;
//     std::set<std::string>  override_set;

//     int arg_i = 2;
//     // Optional positional output path (must not start with '--')
//     if (arg_i < argc && argv[arg_i][0] != '-') {
//         outp = argv[arg_i++];
//     } else {
//         outp = inp.string() + ".lua";
//     }
//     // Remaining flags
//     while (arg_i < argc) {
//         std::string flag = argv[arg_i++];
//         if (flag == "--override" && arg_i < argc) {
//             override_set.insert(argv[arg_i++]);
//         } else {
//             std::cerr << "Warning: unknown argument '" << flag << "'\n";
//         }
//     }
//     try {
//         std::cerr << "[1/5] Reading file: " << inp << "\n";
//         std::string src = read_file(inp);
//         std::cerr << "[2/5] Read " << src.size() << " bytes\n";

//         Tokenizer tz(std::move(src));
//         std::cerr << "[3/5] Tokenizing...\n";
//         auto toks = tz.tokenize();
//         std::cerr << "[3/5] Got " << toks.size() << " tokens\n";

//         Emitter em(std::move(toks));
//         for (const auto& name : override_set)
//             em.add_override(name);
//         std::cerr << "[4/5] Emitting Lua...\n";
//         std::string lua = em.emit();
//         std::cerr << "[4/5] Emitted " << lua.size() << " bytes of Lua\n";

//         std::cerr << "[5/5] Writing: " << outp << "\n";
//         std::ofstream out(outp);
//         if (!out) throw std::runtime_error("Cannot write: " + outp.string());
//         out << lua;

//         std::cout << "Written: " << outp << "\n";
//         std::cerr << "[5/5] Done\n";
//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << "\n";
//         return 1;
//     }

//     auto t_end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed = t_end - t_start;
//     std::cerr << "Elapsed time: " << elapsed.count() << " seconds\n";

//     PROCESS_MEMORY_COUNTERS pmc;
//     if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
//         std::cerr << "Memory used: " << pmc.PeakWorkingSetSize / 1024 << " KB\n";

//     return 0;
// }
