#!/usr/bin/env python3
"""
Make the CMake config files of the ROS 2 install in the target filesystem
usable for cross-compilation.

Packages installed with apt inside the targetfs (ROS 2 Foxy debs, but also
some system packages) export absolute paths such as
    /usr/lib/aarch64-linux-gnu/libtinyxml2.so     (fastrtps-targets.cmake)
    /usr/include/eigen3                            (orocos_kdl, tf2_eigen, ...)
When cross-compiling these point to the HOST filesystem. This is the same
problem the ROS1 README fixes by hand-editing tf2_geometry_msgsConfig.cmake,
done here for every file: the paths are prefixed with ${CMAKE_SYSROOT} (set by
Toolchain-V5L-foxy.cmake), so they keep working for native builds too
(CMAKE_SYSROOT is empty there).

Each modified file is saved next to itself as <file>.backup first, use
--restore to put the originals back.

Usage:
    python3 fix_sysroot_cmake_paths.py <SYSROOT> [--restore] [--dry-run]
"""
import argparse
import re
import shutil
from pathlib import Path

# Directories (inside the sysroot) whose *.cmake files get fixed
SEARCH_DIRS = [
    'opt/ros/foxy',
    'usr/lib/aarch64-linux-gnu/cmake',
    'usr/lib/cmake',
    'usr/share',
]

# Absolute target paths that must be looked up inside the sysroot. Only
# matched at the start of a path (after a quote, ; , ( or whitespace), so
# already fixed paths like "${CMAKE_SYSROOT}/usr/include" are left alone.
PATH_PATTERN = re.compile(
    r'(?<=[\s";(])'
    r'(/usr/lib/aarch64-linux-gnu|/lib/aarch64-linux-gnu|/usr/include|/opt/ros/foxy)'
    r'(?=[/\s";)]|$)', re.MULTILINE)


def cmake_files(sysroot):
    for search_dir in SEARCH_DIRS:
        base = sysroot / search_dir
        if not base.is_dir():
            continue
        for path in base.rglob('*.cmake'):
            if path.is_file() and not path.is_symlink():
                yield path


def fix(sysroot, dry_run):
    fixed = 0
    for path in cmake_files(sysroot):
        try:
            text = path.read_text()
        except UnicodeDecodeError:
            continue
        new_text, count = PATH_PATTERN.subn(r'${CMAKE_SYSROOT}\1', text)
        if count == 0:
            continue
        fixed += 1
        print(f'{"[dry-run] " if dry_run else ""}{path.relative_to(sysroot)}: {count} path(s)')
        if dry_run:
            continue
        backup = path.with_name(path.name + '.backup')
        if not backup.exists():
            shutil.copy2(path, backup)
        path.write_text(new_text)
    print(f'\nFixed {fixed} file(s)')


def restore(sysroot):
    restored = 0
    for search_dir in SEARCH_DIRS:
        base = sysroot / search_dir
        if not base.is_dir():
            continue
        for backup in base.rglob('*.cmake.backup'):
            original = backup.with_name(backup.name[:-len('.backup')])
            shutil.move(str(backup), str(original))
            print(f'Restored: {original.relative_to(sysroot)}')
            restored += 1
    print(f'\nRestored {restored} file(s)')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('sysroot', type=Path, help='target filesystem, e.g. ~/nvidia_ws/target_fs')
    parser.add_argument('--restore', action='store_true', help='restore the *.backup files')
    parser.add_argument('--dry-run', action='store_true', help='only print what would be changed')
    args = parser.parse_args()

    sysroot = args.sysroot.expanduser().resolve()
    if not (sysroot / 'opt/ros/foxy').is_dir():
        parser.error(f'{sysroot}/opt/ros/foxy not found - install ROS 2 Foxy in the targetfs first')

    if args.restore:
        restore(sysroot)
    else:
        fix(sysroot, args.dry_run)


if __name__ == '__main__':
    main()
