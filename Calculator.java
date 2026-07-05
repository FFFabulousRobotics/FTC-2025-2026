import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Minimalist Turing-complete calculator.
 *
 * Supports a tiny Lisp-like language: arithmetic, variables, lambdas,
 * recursion, conditionals and while-loops.  Together these primitives
 * make the calculator Turing complete.
 *
 * Example session:
 *     > (+ 1 2 3)
 *     6
 *     > (define (fact n) (if (= n 0) 1 (* n (fact (- n 1)))))
 *     ok
 *     > (fact 10)
 *     3628800
 *     > (define i 0)
 *     ok
 *     > (while (< i 5) (begin (print i) (set! i (+ i 1))))
 *     0 1 2 3 4 0
 */
public class Calculator {

    // ---------- AST ----------
    interface Node {
        Object eval(Env env);
    }

    static final class NumberNode implements Node {
        final double value;
        NumberNode(double v) { this.value = v; }
        public Object eval(Env env) { return value; }
    }

    static final class StringNode implements Node {
        final String value;
        StringNode(String v) { this.value = v; }
        public Object eval(Env env) { return value; }
    }

    static final class SymbolNode implements Node {
        final String name;
        SymbolNode(String n) { this.name = n; }
        public Object eval(Env env) {
            if (!env.has(name)) {
                throw new EvalException("Undefined symbol: " + name);
            }
            return env.get(name);
        }
    }

    static final class ListNode implements Node {
        final List<Node> elements;
        ListNode(List<Node> e) { this.elements = e; }
        public Object eval(Env env) {
            if (elements.isEmpty()) {
                throw new EvalException("Cannot evaluate empty list");
            }
            Node head = elements.get(0);
            if (!(head instanceof SymbolNode)) {
                // Direct invocation of a lambda: ((lambda (x) ...) arg)
                Object fn = head.eval(env);
                List<Object> args = new ArrayList<>();
                for (int i = 1; i < elements.size(); i++) {
                    args.add(elements.get(i).eval(env));
                }
                return apply(fn, args);
            }
            String op = ((SymbolNode) head).name;
            return applySpecial(op, elements, env);
        }

        private Object applySpecial(String op, List<Node> elems, Env env) {
            switch (op) {
                case "define":    return evalDefine(elems, env);
                case "set!":      return evalSet(elems, env);
                case "lambda":    return evalLambda(elems, env);
                case "if":        return evalIf(elems, env);
                case "begin":     return evalBegin(elems, env);
                case "while":     return evalWhile(elems, env);
                case "quote":     return evalQuote(elems);
                case "and":       return evalAnd(elems, env);
                case "or":        return evalOr(elems, env);
                case "not":       return truthy(elems.get(1).eval(env)) ? 0.0 : 1.0;
                case "print":     return evalPrint(elems, env);
                default: {
                    // Built-in arithmetic / comparison or user function
                    List<Object> args = new ArrayList<>();
                    for (int i = 1; i < elems.size(); i++) {
                        args.add(elems.get(i).eval(env));
                    }
                    Object fn;
                    if (BUILTINS.containsKey(op)) {
                        fn = BUILTINS.get(op);
                    } else if (env.has(op)) {
                        fn = env.get(op);
                    } else {
                        throw new EvalException("Unknown operator: " + op);
                    }
                    return apply(fn, args);
                }
            }
        }

        private Object evalDefine(List<Node> elems, Env env) {
            Node target = elems.get(1);
            if (target instanceof ListNode) {
                // (define (name args...) body)
                ListNode sig = (ListNode) target;
                String name = ((SymbolNode) sig.elements.get(0)).name;
                List<String> params = new ArrayList<>();
                for (int i = 1; i < sig.elements.size(); i++) {
                    params.add(((SymbolNode) sig.elements.get(i)).name);
                }
                Node body = elems.get(2);
                env.put(name, new Lambda(params, body, env));
            } else {
                String name = ((SymbolNode) target).name;
                env.put(name, elems.get(2).eval(env));
            }
            return "ok";
        }

        private Object evalSet(List<Node> elems, Env env) {
            String name = ((SymbolNode) elems.get(1)).name;
            Object value = elems.get(2).eval(env);
            env.setExisting(name, value);
            return "ok";
        }

        private Object evalLambda(List<Node> elems, Env env) {
            ListNode paramsNode = (ListNode) elems.get(1);
            List<String> params = new ArrayList<>();
            for (Node n : paramsNode.elements) {
                params.add(((SymbolNode) n).name);
            }
            Node body = elems.get(2);
            return new Lambda(params, body, env);
        }

        private Object evalIf(List<Node> elems, Env env) {
            Object cond = elems.get(1).eval(env);
            if (truthy(cond)) {
                return elems.get(2).eval(env);
            } else if (elems.size() > 3) {
                return elems.get(3).eval(env);
            }
            return 0.0;
        }

        private Object evalBegin(List<Node> elems, Env env) {
            Object last = 0.0;
            for (int i = 1; i < elems.size(); i++) {
                last = elems.get(i).eval(env);
            }
            return last;
        }

        private Object evalWhile(List<Node> elems, Env env) {
            Node cond = elems.get(1);
            Node body = elems.get(2);
            Object last = 0.0;
            int guard = 0;
            while (truthy(cond.eval(env))) {
                last = body.eval(env);
                if (++guard > 1_000_000) {
                    throw new EvalException("Loop iteration limit exceeded (1,000,000)");
                }
            }
            return last;
        }

        private Object evalQuote(List<Node> elems) {
            return quoteToString(elems.get(1));
        }

        private Object evalAnd(List<Node> elems, Env env) {
            Object last = 1.0;
            for (int i = 1; i < elems.size(); i++) {
                last = elems.get(i).eval(env);
                if (!truthy(last)) return 0.0;
            }
            return last;
        }

        private Object evalOr(List<Node> elems, Env env) {
            for (int i = 1; i < elems.size(); i++) {
                Object v = elems.get(i).eval(env);
                if (truthy(v)) return v;
            }
            return 0.0;
        }

        private Object evalPrint(List<Node> elems, Env env) {
            StringBuilder sb = new StringBuilder();
            for (int i = 1; i < elems.size(); i++) {
                if (i > 1) sb.append(' ');
                sb.append(format(elems.get(i).eval(env)));
            }
            System.out.print(sb.toString() + " ");
            return 0.0;
        }
    }

    // ---------- Lambda / Closure ----------
    static final class Lambda {
        final List<String> params;
        final Node body;
        final Env closure;
        Lambda(List<String> params, Node body, Env closure) {
            this.params = params;
            this.body = body;
            this.closure = closure;
        }
    }

    // ---------- Environment ----------
    static class Env {
        final Env parent;
        final Map<String, Object> vars = new HashMap<>();

        Env() { this.parent = null; }
        Env(Env parent) { this.parent = parent; }

        void put(String name, Object value) { vars.put(name, value); }

        boolean has(String name) {
            if (vars.containsKey(name)) return true;
            return parent != null && parent.has(name);
        }

        Object get(String name) {
            if (vars.containsKey(name)) return vars.get(name);
            if (parent != null) return parent.get(name);
            throw new EvalException("Undefined symbol: " + name);
        }

        void setExisting(String name, Object value) {
            if (vars.containsKey(name)) { vars.put(name, value); return; }
            if (parent != null) { parent.setExisting(name, value); return; }
            throw new EvalException("Cannot set! undefined variable: " + name);
        }
    }

    // ---------- Built-in operators ----------
    interface Builtin { Object apply(List<Object> args); }

    static final Map<String, Builtin> BUILTINS = new HashMap<>();
    static {
        BUILTINS.put("+", args -> { double r = 0; for (Object a : args) r += num(a); return r; });
        BUILTINS.put("-", args -> {
            if (args.size() == 1) return -num(args.get(0));
            double r = num(args.get(0));
            for (int i = 1; i < args.size(); i++) r -= num(args.get(i));
            return r;
        });
        BUILTINS.put("*", args -> { double r = 1; for (Object a : args) r *= num(a); return r; });
        BUILTINS.put("/", args -> {
            if (args.size() == 1) return 1.0 / num(args.get(0));
            double r = num(args.get(0));
            for (int i = 1; i < args.size(); i++) {
                double d = num(args.get(i));
                if (d == 0) throw new EvalException("Division by zero");
                r /= d;
            }
            return r;
        });
        BUILTINS.put("%", args -> num(args.get(0)) % num(args.get(1)));
        BUILTINS.put("=",  args -> num(args.get(0)) == num(args.get(1)) ? 1.0 : 0.0);
        BUILTINS.put("!=", args -> num(args.get(0)) != num(args.get(1)) ? 1.0 : 0.0);
        BUILTINS.put("<",  args -> num(args.get(0)) <  num(args.get(1)) ? 1.0 : 0.0);
        BUILTINS.put(">",  args -> num(args.get(0)) >  num(args.get(1)) ? 1.0 : 0.0);
        BUILTINS.put("<=", args -> num(args.get(0)) <= num(args.get(1)) ? 1.0 : 0.0);
        BUILTINS.put(">=", args -> num(args.get(0)) >= num(args.get(1)) ? 1.0 : 0.0);
    }

    static Object apply(Object fn, List<Object> args) {
        if (fn instanceof Builtin) return ((Builtin) fn).apply(args);
        if (fn instanceof Lambda) {
            Lambda lam = (Lambda) fn;
            if (args.size() != lam.params.size()) {
                throw new EvalException("Arity mismatch: expected "
                        + lam.params.size() + " got " + args.size());
            }
            Env callEnv = new Env(lam.closure);
            for (int i = 0; i < lam.params.size(); i++) {
                callEnv.put(lam.params.get(i), args.get(i));
            }
            return lam.body.eval(callEnv);
        }
        throw new EvalException("Not callable: " + format(fn));
    }

    // ---------- Helpers ----------
    static double num(Object o) {
        if (o instanceof Double) return (Double) o;
        if (o instanceof Number) return ((Number) o).doubleValue();
        throw new EvalException("Expected number, got " + format(o));
    }

    static boolean truthy(Object o) {
        if (o instanceof Double) return (Double) o != 0;
        if (o instanceof Boolean) return (Boolean) o;
        if (o == null) return false;
        return true;
    }

    static String format(Object o) {
        if (o instanceof Double) {
            double d = (Double) o;
            if (d == Math.floor(d) && !Double.isInfinite(d)) {
                return Long.toString((long) d);
            }
            return Double.toString(d);
        }
        if (o instanceof Builtin) return "<builtin>";
        if (o instanceof Lambda) return "<lambda>";
        if (o == null) return "nil";
        return o.toString();
    }

    static String quoteToString(Node n) {
        if (n instanceof NumberNode) return format(((NumberNode) n).value);
        if (n instanceof StringNode) return ((StringNode) n).value;
        if (n instanceof SymbolNode) return ((SymbolNode) n).name;
        if (n instanceof ListNode) {
            StringBuilder sb = new StringBuilder("(");
            List<Node> els = ((ListNode) n).elements;
            for (int i = 0; i < els.size(); i++) {
                if (i > 0) sb.append(' ');
                sb.append(quoteToString(els.get(i)));
            }
            return sb.append(')').toString();
        }
        return "?";
    }

    // ---------- Parser ----------
    static class Parser {
        final String src;
        int pos = 0;
        Parser(String src) { this.src = src; }

        List<Node> parseAll() {
            List<Node> nodes = new ArrayList<>();
            skipWs();
            while (pos < src.length()) {
                nodes.add(parseExpr());
                skipWs();
            }
            return nodes;
        }

        Node parseExpr() {
            skipWs();
            if (pos >= src.length()) throw new EvalException("Unexpected end of input");
            char c = src.charAt(pos);
            if (c == '(') return parseList();
            if (c == ')') throw new EvalException("Unexpected )");
            if (c == '"') return parseString();
            return parseAtom();
        }

        Node parseList() {
            expect('(');
            List<Node> elems = new ArrayList<>();
            skipWs();
            while (pos < src.length() && src.charAt(pos) != ')') {
                elems.add(parseExpr());
                skipWs();
            }
            expect(')');
            return new ListNode(elems);
        }

        Node parseString() {
            expect('"');
            StringBuilder sb = new StringBuilder();
            while (pos < src.length() && src.charAt(pos) != '"') {
                char c = src.charAt(pos++);
                if (c == '\\' && pos < src.length()) {
                    char n = src.charAt(pos++);
                    switch (n) {
                        case 'n': sb.append('\n'); break;
                        case 't': sb.append('\t'); break;
                        default:  sb.append(n);
                    }
                } else {
                    sb.append(c);
                }
            }
            expect('"');
            return new StringNode(sb.toString());
        }

        Node parseAtom() {
            int start = pos;
            while (pos < src.length()) {
                char c = src.charAt(pos);
                if (Character.isWhitespace(c) || c == '(' || c == ')') break;
                pos++;
            }
            String token = src.substring(start, pos);
            try {
                return new NumberNode(Double.parseDouble(token));
            } catch (NumberFormatException e) {
                return new SymbolNode(token);
            }
        }

        void skipWs() {
            while (pos < src.length()) {
                char c = src.charAt(pos);
                if (c == ';') {
                    while (pos < src.length() && src.charAt(pos) != '\n') pos++;
                } else if (Character.isWhitespace(c)) {
                    pos++;
                } else {
                    break;
                }
            }
        }

        void expect(char c) {
            if (pos >= src.length() || src.charAt(pos) != c) {
                throw new EvalException("Expected '" + c + "' at position " + pos);
            }
            pos++;
        }
    }

    // ---------- Errors ----------
    static class EvalException extends RuntimeException {
        EvalException(String msg) { super(msg); }
    }

    // ---------- REPL ----------
    public static void main(String[] args) throws IOException {
        System.out.println("Minimal Turing-complete Calculator");
        System.out.println("Type :help for help, :quit to exit.");
        Env global = new Env();
        BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
        StringBuilder buffer = new StringBuilder();
        while (true) {
            System.out.print(buffer.length() == 0 ? "> " : ". ");
            String line = reader.readLine();
            if (line == null) break;
            String trimmed = line.trim();
            if (trimmed.equals(":quit")) break;
            if (trimmed.equals(":help")) { printHelp(); continue; }
            buffer.append(line).append('\n');
            int open = countUnmatched(buffer.toString());
            if (open > 0) continue; // wait for more input
            String src = buffer.toString();
            buffer.setLength(0);
            try {
                List<Node> nodes = new Parser(src).parseAll();
                for (Node n : nodes) {
                    Object result = n.eval(global);
                    if (!"ok".equals(result) || isReplEcho(n)) {
                        System.out.println(format(result));
                    }
                }
            } catch (EvalException e) {
                System.out.println("error: " + e.getMessage());
            } catch (Exception e) {
                System.out.println("error: " + e.getMessage());
            }
        }
    }

    static boolean isReplEcho(Node n) {
        // Always print results of expressions except for define/set! (which return "ok")
        return !(n instanceof ListNode) ||
                ((ListNode) n).elements.isEmpty() ||
                !(((ListNode) n).elements.get(0) instanceof SymbolNode) ||
                !isSilentOp(((SymbolNode) ((ListNode) n).elements.get(0)).name);
    }

    static boolean isSilentOp(String op) {
        return op.equals("define") || op.equals("set!");
    }

    static int countUnmatched(String s) {
        int depth = 0;
        boolean inStr = false;
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if (inStr) {
                if (c == '"') inStr = false;
                continue;
            }
            if (c == '"') inStr = true;
            else if (c == '(') depth++;
            else if (c == ')') depth--;
        }
        return depth;
    }

    static void printHelp() {
        System.out.println(
            "Operations:\n" +
            "  (+ 1 2 3)        (- 10 3 2)   (* 2 3 4)   (/ 100 5)   (% 10 3)\n" +
            "  (= a b) (!= a b) (< a b) (> a b) (<= a b) (>= a b)\n" +
            "  (and a b c) (or a b) (not x)\n" +
            "Variables:\n" +
            "  (define x 5)            (set! x (+ x 1))\n" +
            "Functions / lambda:\n" +
            "  (define (sq n) (* n n))  ((lambda (x) (* x x)) 5)\n" +
            "Control flow:\n" +
            "  (if cond then else)  (begin e1 e2 ...)  (while cond body)\n" +
            "  (quote (1 2 3))      (print x y z)\n" +
            "Examples:\n" +
            "  (define (fact n) (if (= n 0) 1 (* n (fact (- n 1)))))\n" +
            "  (fact 10)   ; => 3628800\n" +
            "  (define i 0) (while (< i 5) (begin (print i) (set! i (+ i 1))))\n");
    }
}
