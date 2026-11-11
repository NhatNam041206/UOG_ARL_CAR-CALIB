#!/usr/bin/env python3
"""Deep-merge a sweep variant onto nav2_params.yaml.

    python3 apply_overrides.py base_nav2_params.yaml variant.yaml out.yaml

A variant file holds only the keys it changes, nested exactly like
nav2_params.yaml (node -> ros__parameters -> key). Keys that do not exist
in the base are an error, so a typo cannot silently test nothing.
"""
import sys

import yaml


def merge(base, over, path=""):
    for k, v in over.items():
        here = f"{path}.{k}" if path else str(k)
        if k not in base:
            raise SystemExit(f"variant key {here} is not in the base params")
        if isinstance(v, dict) and isinstance(base[k], dict):
            merge(base[k], v, here)
        else:
            base[k] = v


def main(base_path, variant_path, out_path):
    with open(base_path, encoding="utf-8") as fh:
        base = yaml.safe_load(fh)
    with open(variant_path, encoding="utf-8") as fh:
        variant = yaml.safe_load(fh) or {}
    variant.pop("description", None)
    merge(base, variant.get("nav2_params") or {})
    with open(out_path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(base, fh, sort_keys=False)


if __name__ == "__main__":
    main(*sys.argv[1:4])
