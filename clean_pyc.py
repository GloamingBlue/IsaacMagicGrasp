#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import shutil

def clean_pyc(path):
    """
    删除指定路径下所有 .pyc 文件和 __pycache__ 目录。
    """
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.endswith(".pyc"):
                file_path = os.path.join(root, file)
                print(f"[删除文件] {file_path}")
                try:
                    os.remove(file_path)
                except OSError as e:
                    print(f"删除失败: {e}")

        for d in dirs:
            if d == '__pycache__':
                dir_path = os.path.join(root, d)
                print(f"[删除目录] {dir_path}")
                try:
                    shutil.rmtree(dir_path)
                except OSError as e:
                    print(f"删除失败: {e}")

if __name__ == "__main__":
    # 支持传入路径作为参数，否则清除当前目录
    base = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    clean_pyc(base)
