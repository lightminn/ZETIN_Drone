"""Build a deterministic, allowlisted tar.gz Oracle web release."""

from __future__ import annotations

import argparse
import gzip
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


def _git_command(repo_root: Path, *arguments: str) -> list[str]:
    return ["git", "--literal-pathspecs", "-C", str(repo_root), *arguments]


def _git_bytes(repo_root: Path, *arguments: str) -> bytes:
    result = subprocess.run(_git_command(repo_root, *arguments), capture_output=True)
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise BuildError(message or "git command failed")
    return result.stdout


def _git(repo_root: Path, *arguments: str) -> str:
    return _git_bytes(repo_root, *arguments).decode("utf-8")


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
    output = _git_bytes(repo_root, "ls-files", "--stage", "-z", "--", relative)
    entries = output.split(b"\0")
    if len(entries) != 2 or entries[1] or b"\t" not in entries[0]:
        raise BuildError(f"{label} is not tracked: {relative}")
    metadata, tracked_path = entries[0].split(b"\t", 1)
    fields = metadata.split()
    if (
        len(fields) != 3
        or fields[0] not in {b"100644", b"100755"}
        or fields[2] != b"0"
        or tracked_path != os.fsencode(relative)
    ):
        raise BuildError(f"{label} is not a tracked regular file: {relative}")


def _ensure_clean(repo_root: Path, paths: list[str], source_commit: str) -> None:
    result = subprocess.run(_git_command(repo_root, "diff", "--quiet", source_commit, "--", *paths))
    if result.returncode == 1:
        raise BuildError("site config or allowlisted source is dirty")
    if result.returncode != 0:
        raise BuildError("cannot inspect allowlisted source cleanliness")


def _commit_blob_oid(repo_root: Path, source_commit: str, relative: str, label: str) -> str:
    output = _git_bytes(repo_root, "ls-tree", "-z", source_commit, "--", relative)
    entries = output.split(b"\0")
    if len(entries) != 2 or entries[1] or b"\t" not in entries[0]:
        raise BuildError(f"{label} is missing from source commit: {relative}")
    metadata, tracked_path = entries[0].split(b"\t", 1)
    fields = metadata.split()
    if (
        len(fields) != 3
        or fields[0] not in {b"100644", b"100755"}
        or fields[1] != b"blob"
        or tracked_path != os.fsencode(relative)
    ):
        raise BuildError(f"{label} is not a regular file in source commit: {relative}")
    object_id = fields[2]
    if len(object_id) not in {40, 64} or any(byte not in b"0123456789abcdef" for byte in object_id):
        raise BuildError(f"cannot resolve {label} Git blob: {relative}")
    return object_id.decode("ascii")


def _materialize_blob(repo_root: Path, object_id: str, destination: Path, label: str) -> None:
    try:
        with destination.open("xb") as stream:
            result = subprocess.run(
                _git_command(repo_root, "cat-file", "blob", object_id),
                stdout=stream,
                stderr=subprocess.PIPE,
            )
    except OSError as error:
        raise BuildError(f"cannot snapshot {label}") from error
    if result.returncode != 0:
        destination.unlink(missing_ok=True)
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise BuildError(message or f"cannot read {label} Git blob")


def _snapshot_commit_file(
    repo_root: Path,
    source_commit: str,
    relative: str,
    destination: Path,
    label: str,
) -> None:
    object_id = _commit_blob_oid(repo_root, source_commit, relative, label)
    _materialize_blob(repo_root, object_id, destination, label)


def _member_metadata(source_files: dict[str, Path], manifest: SiteManifest) -> list[dict[str, object]]:
    metadata: list[dict[str, object]] = []
    for entry in manifest.files:
        source = source_files[entry.source]
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


def _tar_add_file(archive: tarfile.TarFile, name: str, source: Path, mode: int) -> None:
    info = tarfile.TarInfo(name=name)
    info.size = source.stat().st_size
    info.mode = mode
    info.uid = 0
    info.gid = 0
    info.uname = "root"
    info.gname = "root"
    info.mtime = 0
    info.type = tarfile.REGTYPE
    with source.open("rb") as stream:
        archive.addfile(info, stream)


def _write_archive(output: Path, source_files: dict[str, Path], manifest: SiteManifest, release_bytes: bytes) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{output.name}.", dir=output.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w", format=tarfile.USTAR_FORMAT) as archive:
                    _tar_add_bytes(archive, "release.json", release_bytes, 0o444)
                    for entry in manifest.files:
                        _tar_add_file(
                            archive,
                            entry.destination,
                            source_files[entry.source],
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
    source_commit = _git(repo_root, "rev-parse", "--verify", "HEAD^{commit}").strip()
    with tempfile.TemporaryDirectory(prefix="oracle-web-release-snapshot-") as snapshot_name:
        snapshot_root = Path(snapshot_name)
        config_snapshot = snapshot_root / "site-config.json"
        _snapshot_commit_file(repo_root, source_commit, config_relative, config_snapshot, "site config")
        try:
            manifest = load_site_manifest(config_snapshot)
        except ManifestError as error:
            raise BuildError(str(error)) from error
        source_paths = [entry.source for entry in manifest.files]
        for source in source_paths:
            _ensure_tracked_regular(repo_root, repo_root / source, source, "allowlisted source")
        _ensure_clean(repo_root, [config_relative, *source_paths], source_commit)
        source_files: dict[str, Path] = {}
        for index, source in enumerate(source_paths):
            if source in source_files:
                continue
            snapshot = snapshot_root / f"source-{index}"
            _snapshot_commit_file(repo_root, source_commit, source, snapshot, "allowlisted source")
            source_files[source] = snapshot
        members = _member_metadata(source_files, manifest)
        _write_archive(
            output.absolute(),
            source_files,
            manifest,
            _release_metadata(manifest, release_id, source_commit, members),
        )
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
