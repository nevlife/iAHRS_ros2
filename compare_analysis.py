#!/usr/bin/env python3

import re
import os
from pathlib import Path

class CodeAnalyzer:
    def __init__(self, filepath):
        self.filepath = filepath
        with open(filepath, 'r', encoding='utf-8') as f:
            self.content = f.read()
            self.lines = self.content.split('\n')

    def count_lines(self):
        total = len(self.lines)
        code_lines = 0
        comment_lines = 0
        blank_lines = 0

        in_multiline_comment = False

        for line in self.lines:
            stripped = line.strip()

            if not stripped:
                blank_lines += 1
                continue

            if self.filepath.endswith('.cpp'):
                if stripped.startswith('/*'):
                    in_multiline_comment = True
                if in_multiline_comment:
                    comment_lines += 1
                    if '*/' in stripped:
                        in_multiline_comment = False
                    continue
                if stripped.startswith('//'):
                    comment_lines += 1
                    continue
            elif self.filepath.endswith('.py'):
                if stripped.startswith('#'):
                    comment_lines += 1
                    continue
                if stripped.startswith('"""') or stripped.startswith("'''"):
                    comment_lines += 1
                    continue

            code_lines += 1

        return {
            'total': total,
            'code': code_lines,
            'comments': comment_lines,
            'blank': blank_lines
        }

    def count_functions(self):
        if self.filepath.endswith('.cpp'):
            pattern = r'^\s*(void|int|bool|double|char\*|auto|std::.*?)\s+(\w+)\s*\('
        else:
            pattern = r'^\s*def\s+(\w+)\s*\('

        functions = []
        for line in self.lines:
            match = re.search(pattern, line)
            if match:
                if self.filepath.endswith('.cpp'):
                    functions.append(match.group(2))
                else:
                    functions.append(match.group(1))

        return functions

    def count_classes(self):
        if self.filepath.endswith('.cpp'):
            pattern = r'^\s*class\s+(\w+)'
        else:
            pattern = r'^\s*class\s+(\w+)'

        classes = []
        for line in self.lines:
            match = re.search(pattern, line)
            if match:
                classes.append(match.group(1))

        return classes

    def count_includes_imports(self):
        if self.filepath.endswith('.cpp'):
            pattern = r'^\s*#include\s+[<"](.+?)[>"]'
        else:
            pattern = r'^\s*(import|from)\s+([\w\.]+)'

        deps = []
        for line in self.lines:
            match = re.search(pattern, line)
            if match:
                if self.filepath.endswith('.cpp'):
                    deps.append(match.group(1))
                else:
                    deps.append(match.group(2))

        return deps

    def analyze_complexity(self):
        if_count = len(re.findall(r'\bif\s*\(', self.content))
        for_count = len(re.findall(r'\bfor\s*\(', self.content))
        while_count = len(re.findall(r'\bwhile\s*\(', self.content))
        try_count = len(re.findall(r'\btry\s*{', self.content)) if self.filepath.endswith('.cpp') else len(re.findall(r'\btry:', self.content))

        return {
            'if_statements': if_count,
            'for_loops': for_count,
            'while_loops': while_count,
            'try_blocks': try_count,
            'cyclomatic_complexity': if_count + for_count + while_count + try_count + 1
        }

def compare_files(cpp_file, py_file):
    print("="*80)
    print("iAHRS Driver C++ vs Python Code Analysis")
    print("="*80)

    cpp_analyzer = CodeAnalyzer(cpp_file)
    py_analyzer = CodeAnalyzer(py_file)

    print("\n1. LINE COUNT COMPARISON")
    print("-"*80)

    cpp_lines = cpp_analyzer.count_lines()
    py_lines = py_analyzer.count_lines()

    print(f"{'Metric':<20} {'C++':<15} {'Python':<15} {'Difference':<15}")
    print("-"*80)
    print(f"{'Total Lines':<20} {cpp_lines['total']:<15} {py_lines['total']:<15} {cpp_lines['total']-py_lines['total']:+<15}")
    print(f"{'Code Lines':<20} {cpp_lines['code']:<15} {py_lines['code']:<15} {cpp_lines['code']-py_lines['code']:+<15}")
    print(f"{'Comment Lines':<20} {cpp_lines['comments']:<15} {py_lines['comments']:<15} {cpp_lines['comments']-py_lines['comments']:+<15}")
    print(f"{'Blank Lines':<20} {cpp_lines['blank']:<15} {py_lines['blank']:<15} {cpp_lines['blank']-py_lines['blank']:+<15}")

    print("\n2. CODE STRUCTURE")
    print("-"*80)

    cpp_funcs = cpp_analyzer.count_functions()
    py_funcs = py_analyzer.count_functions()
    cpp_classes = cpp_analyzer.count_classes()
    py_classes = py_analyzer.count_classes()

    print(f"{'Metric':<20} {'C++':<15} {'Python':<15}")
    print("-"*80)
    print(f"{'Classes':<20} {len(cpp_classes):<15} {len(py_classes):<15}")
    print(f"{'Functions/Methods':<20} {len(cpp_funcs):<15} {len(py_funcs):<15}")

    print("\n  C++ Functions:", ', '.join(cpp_funcs))
    print("  Python Methods:", ', '.join(py_funcs))

    print("\n3. DEPENDENCIES")
    print("-"*80)

    cpp_deps = cpp_analyzer.count_includes_imports()
    py_deps = py_analyzer.count_includes_imports()

    print(f"C++ includes ({len(cpp_deps)}):")
    for dep in cpp_deps:
        print(f"  - {dep}")

    print(f"\nPython imports ({len(py_deps)}):")
    for dep in py_deps:
        print(f"  - {dep}")

    print("\n4. CODE COMPLEXITY")
    print("-"*80)

    cpp_complexity = cpp_analyzer.analyze_complexity()
    py_complexity = py_analyzer.analyze_complexity()

    print(f"{'Metric':<25} {'C++':<15} {'Python':<15}")
    print("-"*80)
    print(f"{'If Statements':<25} {cpp_complexity['if_statements']:<15} {py_complexity['if_statements']:<15}")
    print(f"{'For Loops':<25} {cpp_complexity['for_loops']:<15} {py_complexity['for_loops']:<15}")
    print(f"{'While Loops':<25} {cpp_complexity['while_loops']:<15} {py_complexity['while_loops']:<15}")
    print(f"{'Try-Catch Blocks':<25} {cpp_complexity['try_blocks']:<15} {py_complexity['try_blocks']:<15}")
    print(f"{'Cyclomatic Complexity':<25} {cpp_complexity['cyclomatic_complexity']:<15} {py_complexity['cyclomatic_complexity']:<15}")

    print("\n5. FILE SIZE")
    print("-"*80)

    cpp_size = os.path.getsize(cpp_file)
    py_size = os.path.getsize(py_file)

    print(f"{'File':<20} {'Size':<15}")
    print("-"*80)
    print(f"{'C++':<20} {cpp_size:,} bytes")
    print(f"{'Python':<20} {py_size:,} bytes")
    print(f"{'Difference':<20} {cpp_size - py_size:+,} bytes")

    print("\n6. KEY DIFFERENCES")
    print("-"*80)

    print("\nC++ Specific:")
    print("  - Uses POSIX API (termios, fcntl, unistd)")
    print("  - Manual memory management (serial_fd_)")
    print("  - Static typing with explicit types")
    print("  - RAII pattern for resource cleanup")
    print("  - Uses tf2::Quaternion for conversion")

    print("\nPython Specific:")
    print("  - Uses pyserial library")
    print("  - Automatic garbage collection")
    print("  - Dynamic typing")
    print("  - Manual quaternion calculation")
    print("  - Simpler exception handling")

    print("\n7. PERFORMANCE CRITICAL SECTIONS")
    print("-"*80)

    cpp_serial_calls = cpp_analyzer.content.count('send_and_receive')
    py_serial_calls = py_analyzer.content.count('send_and_receive')

    print(f"{'Serial I/O calls':<25} {cpp_serial_calls:<15} {py_serial_calls:<15}")

    cpp_math_ops = cpp_analyzer.content.count('cos') + cpp_analyzer.content.count('sin')
    py_math_ops = py_analyzer.content.count('cos') + py_analyzer.content.count('sin')

    print(f"{'Math operations':<25} {cpp_math_ops:<15} {py_math_ops:<15}")

    print("\n" + "="*80)

if __name__ == '__main__':
    cpp_file = 'src/iahrs_driver.cpp'
    py_file = 'src/iahrs_driver.py'

    if not os.path.exists(cpp_file):
        print(f"Error: {cpp_file} not found")
        exit(1)

    if not os.path.exists(py_file):
        print(f"Error: {py_file} not found")
        exit(1)

    compare_files(cpp_file, py_file)
