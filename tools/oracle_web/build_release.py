"""Build a deterministic, allowlisted tar.gz Oracle web release."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import os
import stat
import subprocess
import tarfile
import tempfile

from .common import sha256_file, validate_release_id
from .site_manifest import ManifestError, SiteManifest, load_site_manifest


class BuildError(RuntimeError):
    """Raised when a release cannot be safely built from the current checkout."""


def _git(repo_root: Path, *arguments: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo_root), *arguments],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise BuildError(result.stderr.strip() or "git command failed")
    return result.stdout


def _repo_root(path: Path) -> Path:
    candidate = path.resolve()
    if not candidate.is_dir():
        raise BuildError("repo root must be a directory")
    return Path(_git(candidate, "rev-parse", "--show-toplevel").strip()).resolve()


def _repo_relative(repo_root: Path, path: Path, label: str) -> tuple[Path, str]:
    try:
        relative = path.absolute().relative_to(repo_root)
    except ValueError as error:
        raise BuildError(f"{label} must be inside repo root") from error
    return repo_root / relative, relative.as_posix()


def _ensure_tracked_regular(repo_root: Path, path: Path, relative: str, label: str) -> None:
    try:
        mode = path.lstat().st_mode
    except OSError as error:
        raise BuildError(f"{label} is missing: {relative}") from error
    if not stat.S_ISREG(mode):
        raise BuildError(f"{label} must be a regular file: {relative}")
    output = _git(repo_root, "ls-files", "-s", "--", relative).strip()
    entries = output.splitlines()
    if len(entries) != 1:
        raise BuildError(f"{label} is not tracked: {relative}")
    metadata, tracked_path = entries[0].split("\t", 1)
    fields = metadata.split()
    if len(fields) != 3 or fields[0] not in {"100644", "100755"} or fields[2] != "0" or tracked_path != relative:
        raise BuildError(f"{label} is not a tracked regular file: {relative}")


def _ensure_clean(repo_root: Path, paths: list[str]) -> None:
    result = subprocess.run(["git", "-C", str(repo_root), "diff", "--quiet", "HEAD", "--", *paths])
    if result.returncode == 1:
        raise BuildError("site config or allowlisted source is dirty")
    if result.returncode != 0:
        raise BuildError("cannot inspect allowlisted source cleanliness")


def _member_metadata(repo_root: Path, manifest: SiteManifest) -> list[dict[str, object]]:
    metadata: list[dict[str, object]] = []
    for entry in manifest.files:
        source = repo_root / entry.source
        metadata.append(
            {
                "path": entry.destination,
                "sha256": sha256_file(source),
                "size": source.stat().st_size,
                "mode": 0o555 if entry.destination == "run" else 0o444,
            }
        )
    return metadata


def _release_metadata(manifest: SiteManifest, release_id: str, source_commit: str, members: list[dict[str, object]]) -> bytes:
    release: dict[str, object] = {
        "schema_version": 1,
        "site": manifest.site,
        "release_id": release_id,
        "source_commit": source_commit,
        "server_name": manifest.server_name,
        "public_ipv4": manifest.public_ipv4,
        "https_health_paths": list(manifest.https_health_paths),
        "members": members,
    }
    if manifest.backend is not None:
        release["backend"] = {"port": manifest.backend.port, "health_path": manifest.backend.health_path}
    return (json.dumps(release, sort_keys=True, indent=2) + "\n").encode("utf-8")


def _tar_add_bytes(archive: tarfile.TarFile, name: str, content: bytes, mode: int) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = len(content)
    info.mode = mode
    info.uid = 0
    info.gid = 0
    info.uname = "root"
    info.gname = "root"
    info.mtime = 0
    info.type = tarfile.REGTYPE
    archive.addfile(info, io.BytesIO(content))


def _write_archive(output: Path, repo_root: Path, manifest: SiteManifest, release_bytes: bytes) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{output.name}.", dir=output.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w", format=tarfile.USTAR_FORMAT) as archive:
                    _tar_add_bytes(archive, "release.json", release_bytes, 0o444)
                    for entry in manifest.files:
                        _tar_add_bytes(
                            archive,
                            entry.destination,
                            (repo_root / entry.source).read_bytes(),
                            0o555 if entry.destination == "run" else 0o444,
                        )
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def _write_sidecar(output: Path) -> None:
    digest = sha256_file(output)
    sidecar = Path(str(output) + ".sha256")
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{sidecar.name}.", dir=sidecar.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(f"{digest}  {output.name}\n")
        os.replace(temporary, sidecar)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def build_release(repo_root_argument: Path, config_argument: Path, release_id: str, output: Path) -> None:
    """Validate inputs and create one deterministic release archive and SHA sidecar."""
    try:
        release_id = validate_release_id(release_id)
    except ValueError as error:
        raise BuildError(str(error)) from error
    repo_root = _repo_root(repo_root_argument)
    config_path, config_relative = _repo_relative(repo_root, config_argument, "site config")
    _ensure_tracked_regular(repo_root, config_path, config_relative, "site config")
    try:
        manifest = load_site_manifest(config_path)
    except ManifestError as error:
        raise BuildError(str(error)) from error
    source_paths = [entry.source for entry in manifest.files]
    for source in source_paths:
        _ensure_tracked_regular(repo_root, repo_root / source, source, "allowlisted source")
    _ensure_clean(repo_root, [config_relative, *source_paths])
    source_commit = _git(repo_root, "rev-parse", "HEAD").strip()
    members = _member_metadata(repo_root, manifest)
    _write_archive(output.absolute(), repo_root, manifest, _release_metadata(manifest, release_id, source_commit, members))
    _write_sidecar(output.absolute())


def _arguments() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", required=True, type=Path)
    parser.add_argument("--site-config", required=True, type=Path)
    parser.add_argument("--release-id", required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main() -> None:
    parser = _arguments()
    arguments = parser.parse_args()
    try:
        build_release(arguments.repo_root, arguments.site_config, arguments.release_id, arguments.output)
    except BuildError as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
