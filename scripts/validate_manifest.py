#!/usr/bin/env python3
"""Validate sheppy-manifest.yaml and every profile in profiles/ through
sheppy's own loader, so what loads green here loads on the Jetson.

Run from the repo root:
    uv run --with /path/to/sheppy python scripts/validate_manifest.py

Sheppy has no `validate` verb (rammp-org/sheppy#14), so this reaches into
its loader the way rammp-deployments does. Compose references are resolved
here as well, because sheppy only reads them at launch. The `build` and
`container_name` warnings are tolerated: docker-compose.yml is also used by
plain `docker compose`, and sheppy merely ignores those keys.
"""

import os
import sys

from sheppy.launch.docker.compose import load_service, service_to_docker_args
from sheppy.manifest.loader import load_manifest
from sheppy.profiles.reconcile import reconcile
from sheppy.profiles.store import ProfileStore

TOLERATED = ("compose 'build' is ignored", "compose 'container_name' is ignored")


def check_manifest(path):
    if not os.path.exists(path):
        return None, [f"{path}: not found"]
    result = load_manifest(path)
    problems = [f"{err.location}: {err.message}" for err in result.errors]
    if not result.ok:
        return None, problems
    manifest_dir = os.path.dirname(os.path.abspath(path))
    for node in result.manifest.nodes:
        for alt in node.alternatives:
            if alt.kind != "docker":
                continue
            where = f"{node.name}/{alt.id}"
            if "container" in alt.raw:
                service, base_dir = alt.raw["container"], manifest_dir
            else:
                ref = alt.raw["compose"]
                file = os.path.join(manifest_dir, ref["file"])
                try:
                    service, _ = load_service(file, ref["service"], os.environ)
                except (OSError, KeyError) as e:
                    problems.append(f"{where}: compose service unreadable: {e}")
                    continue
                base_dir = os.path.dirname(file)
            *_, errs, warns = service_to_docker_args(service, base_dir)
            problems += [f"{where}: {e}" for e in errs]
            problems += [f"{where}: warning: {w}" for w in warns
                         if not w.startswith(TOLERATED)]
    return result.manifest, problems


def check_profiles(manifest, profiles_dir):
    store = ProfileStore(profiles_dir)
    names = store.list_profiles()
    problems = [] if names else [f"{profiles_dir}: no profiles found"]
    for name in names:
        loaded = store.load(name)
        if loaded.profile is None:
            problems += [f"profiles/{name}: {e}" for e in loaded.errors]
            continue
        problems += [f"profiles/{name}: {w}"
                     for w in reconcile(loaded.profile, manifest).warnings]
    return names, problems


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    manifest, problems = check_manifest(os.path.join(root, "sheppy-manifest.yaml"))
    names = []
    if manifest is not None:
        names, more = check_profiles(manifest, os.path.join(root, "profiles"))
        problems += more
    for p in problems:
        print(p)
    if problems:
        return 1
    alts = sum(len(n.alternatives) for n in manifest.nodes)
    print(f"ok: {len(manifest.nodes)} nodes, {alts} alternatives, "
          f"{len(names)} profiles ({', '.join(names)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
